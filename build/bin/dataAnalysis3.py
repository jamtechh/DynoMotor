import subprocess
import csv
import matplotlib.pyplot as plt
import os
import re
import random
import math
import shutil

# Initial parameters (starting guesses)
initial_params = {
    'R_motor': 0.15,
    'L_motor': 0.000001,
    'ke_motor': 0.002069,
    'kt_motor': 0.001842,
    'B': 0.000001,
    'T_load': 0.001882
}

volt = 3.5
kv_motor = 3216.11753
latest_run = 0  # Initialize for plotting

# Function to compute RMSE between two lists
def rmse(list1, list2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(list1, list2)) / len(list1))

# Run analysis with specific motor parameters
def runAnalysisWithParams(params, run_id):
    args_list = []
    input_data = {}
    output_data = []

    with open("analysis/noLoad2.csv", newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for header in reader.fieldnames:
            input_data[header] = []
        for row in reader:
            for key in row:
                input_data[key].append(float(row[key]))

    for dcV in input_data["volt"]:
        args = [
            "./my_demo",
            str(dcV),
            str(params["R_motor"]),
            str(params["ke_motor"]),
            str(params["kt_motor"]),
            str(params["B"]),
            str(params["T_load"])
        ]
        result = subprocess.run(args, capture_output=True, text=True)
        lines = result.stdout.strip().splitlines()
        try:
            last_line = lines[-1].strip()
            dcV, rpm, current, R_motor, kv_motor, kt_motor, L_motor = map(float, last_line.split(','))
            output_data.append({
                'volt': dcV,
                'rpm': rpm,
                'current': current,
                'R_motor': R_motor,
                'L_motor': L_motor,
                'kt_motor': kt_motor
            })
        except:
            continue

    # Save simulated data to temp file
    out_path = f"analysis/temp_sim_{run_id}.csv"
    with open(out_path, mode='w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=output_data[0].keys())
        writer.writeheader()
        writer.writerows(output_data)

    return out_path

# Evaluate error between input and simulated data
def computeError(sim_csv_path):
    with open("analysis/noLoad2.csv", newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        input_data = {key: [] for key in reader.fieldnames}
        for row in reader:
            for key in row:
                input_data[key].append(float(row[key]))

    with open(sim_csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        sim_data = {key: [] for key in reader.fieldnames}
        for row in reader:
            for key in row:
                sim_data[key].append(float(row[key]))

    min_len = min(len(input_data["rpm"]), len(sim_data["rpm"]))
    return rmse(input_data["rpm"][:min_len], sim_data["rpm"][:min_len])

# Optimization loop
def optimizeParams(iterations=50, perturb_scale=0.1):
    global latest_run
    best_error = float('inf')
    best_params = initial_params.copy()

    for i in range(iterations):
        # Perturb parameters
        trial_params = {
            key: val * (1 + random.uniform(-perturb_scale, perturb_scale))
            for key, val in best_params.items()
        }

        sim_path = runAnalysisWithParams(trial_params, i)
        error = computeError(sim_path)
        print(f"[Run {i}] RMSE Error: {error:.6f} with params: {trial_params}")

        if error < best_error:
            best_error = error
            best_params = trial_params.copy()
            shutil.copyfile(sim_path, "analysis/motor_output.csv")
            latest_run = i

    print(f"\nBest error: {best_error:.6f}")
    print("Best parameters:")
    for k, v in best_params.items():
        print(f"{k}: {v:.7f}")

def plotData():
    input_data = {}
    with open("analysis/noLoad2.csv", newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for header in reader.fieldnames:
            input_data[header] = []
        for row in reader:
            for key in row:
                input_data[key].append(float(row[key]))

    # Read simulation output CSV
    sim_data = {}
    with open("analysis/motor_output.csv", newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for header in reader.fieldnames:
            sim_data[header] = []
        for row in reader:
            for key in row:
                sim_data[key].append(float(row[key]))

    # Make sure both files have the same length
    min_len = min(len(input_data["volt"]), len(sim_data["current"]))
    voltages = input_data["volt"][:min_len]
    input_currents = input_data["current"][:min_len]
    input_rpm = input_data["rpm"][:min_len]
    sim_currents = sim_data["current"][:min_len]
    sim_rpm = sim_data["rpm"][:min_len]

    fig, axs = plt.subplots(1, 2, figsize=(14, 10))  # (rows, cols)

    axs[0].plot(voltages, input_currents, label="Input Current (A)", marker='o')
    axs[0].plot(voltages, sim_currents, label="Simulated Current (A)", marker='x')
    axs[0].set_xlabel("Voltage (V)")
    axs[0].set_ylabel("Current (A)")
    axs[0].set_title("Input vs Simulated Motor Current")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(voltages, input_rpm, label="Input rpm", marker='o')
    axs[1].plot(voltages, sim_rpm, label="Simulated rpm", marker='x')
    axs[1].set_xlabel("Voltage (V)")
    axs[1].set_ylabel("RPM")
    axs[1].set_title("Input vs Simulated Motor RPM")
    axs[1].grid(True)
    axs[1].legend()

    info = ''
    info += f"volt: {volt:.3f}\n"
    info += f"R_motor: {R_motor:.3f}\n"
    info += f"L_motor: {L_motor:.7f}\n"
    info += f"ke_motor: {ke_motor:.7f}\n"
    info += f"kt_motor: {kt_motor:.7f}\n"
    info += f"B: {B:.7f}\n"
    info += f"T_load: {T_load:.7f}\n"

    fig.text(
        0.35, 0.05,  # X, Y position (from 0 to 1, figure coords)
        info,  # Text to display
        ha='left', va='bottom',  # Align text to corner
        fontsize=12, color='black'
    )

    imageDir = "outputImage/Image_" + str(latest_run) + ".png"
    plt.savefig(imageDir, dpi=300, bbox_inches='tight')

    plt.tight_layout()
    plt.show()

# Run optimization and plot results
optimizeParams(iterations=50)
plotData()