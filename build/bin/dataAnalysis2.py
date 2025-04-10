import subprocess
import csv
import matplotlib.pyplot as plt
import os
import re
import pandas as pd

kv_motor    = 3216.11753

volt        = 3.5
R_motor     = 0.2524
L_motor     = 0.000001
ke_motor    = 0.003104
kt_motor    = 0.001740
B           = 0.000001
T_load      = 0.001688

# Pattern: run_<number>_*.py
pattern = re.compile(r"run_(\d+)_.*\.json$")

latest_run = -1
latest_file = None

# Scan files in the current directory
for filename in os.listdir("outputPlot/"):
    match = pattern.match(filename)
    if match:
        run_num = int(match.group(1))
        if run_num > latest_run:
            latest_run = run_num
            latest_file = filename


def runAnalysis():
    global volt      
    global R_motor   
    global L_motor   
    global ke_motor  
    global kt_motor  
    global B 
    global T_load    

    output_data = []

    input = {}

    with open("analysis/noLoad2.csv", newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for header in reader.fieldnames:
            input[header] = []  # initialize list for each column

        for row in reader:
            print(row)
            for key in row:
                input[key].append(float(row[key]))  # convert to float if numeric


    for dcV in input['volt']:
        # Args to pass to C++ program
        args = [
            "./my_demo",
            str(dcV),
            str(R_motor),
            # str(L_motor),
            str(ke_motor),
            str(kt_motor),
            str(B),
            str(T_load)
        ]
        
        # if dcV >=5:break

        # Run the C++ binary
        result = subprocess.run(args, capture_output=True, text=True)

        # Print all lines
        lines = result.stdout.strip().splitlines()
        for line in lines:
            print(line)

        # Parse the last line
        try:
            last_line = lines[-1].strip()
            dcV, rpm, current, R_motor, kv_motor, kt_motor, L_motor = map(float, last_line.split(','))
                
            print(f"Parsed values -> volt: {dcV}, rpm: {rpm}, current: {current}, L_motor: {L_motor}")

        except ValueError:
            print(f"Failed to parse last line: {last_line}")

        output_data.append({
            'volt': dcV,
            'rpm': rpm,
            'current': current,
            'R_motor': R_motor,
            'L_motor': L_motor,
            'kt_motor': kt_motor
        })

    # Save to CSV if parsing succeeded
    if output_data:
        csv_filename = "analysis/motor_output.csv"
        with open(csv_filename, mode='w', newline='') as csvfile:
            fieldnames = output_data[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(output_data)

        print(f"Saved results to {csv_filename}")


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

runAnalysis()
plotData()

# import copy
# import math
# # import dataAnalysis2.py
# best_error = float('inf')
# best_params = {}
# errors = []

# original_kt = kt_motor
# original_ke = ke_motor

# # Small variations (tune as needed)
# kt_range = 0.001742
# ke_range = [original_ke * (1 + 0.01 * i) for i in range(-5, 6)]  # -5% to +5%

# iteration = 0

# for new_ke in ke_range:
#     iteration += 1
#     print(f"\n=== Iteration {iteration} ke: {new_ke:.7f} ===")

#     # Set the globals
#     # kt_motor = new_kt
#     ke_motor = new_ke

#     # Run the analysis
#     runAnalysis()

#     # Load data to compute error
#     try:
#         with open("analysis/noLoad2.csv", newline='') as f:
#             reader = csv.DictReader(f)
#             input_data = [row for row in reader]
#         with open("analysis/motor_output.csv", newline='') as f:
#             reader = csv.DictReader(f)
#             sim_data = [row for row in reader]
#     except Exception as e:
#         print(f"Error reading CSV: {e}")
#         continue

#     total_error = 0.0
#     count = min(len(input_data), len(sim_data))
#     for i in range(count):
#         try:
#             input_rpm = float(input_data[i]['rpm'])
#             input_current = float(input_data[i]['current'])

#             sim_rpm = float(sim_data[i]['rpm'])
#             sim_current = float(sim_data[i]['current'])

#             # Compute simple absolute error
#             rpm_err = abs(input_rpm - sim_rpm)
#             curr_err = abs(input_current - sim_current)

#             total_error += (rpm_err + curr_err) / 2.0

#         except Exception as e:
#             print(f"Parse error on row {i}: {e}")

#     avg_error = total_error / count
#     errors.append((avg_error, new_ke))
#     print(f"Average error: {avg_error:.5f}")

#     if avg_error < best_error:
#         best_error = avg_error
#         best_params = {'ke_motor': new_ke}

#     plotData()

# print("\n=== DONE ===")
# print(f"Best avg error: {best_error:.5f}")
# print(f"Best parameters: kt_motor = {best_params['kt_motor']:.7f}, ke_motor = {best_params['ke_motor']:.7f}")