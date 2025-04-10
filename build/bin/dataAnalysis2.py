import subprocess
import csv
import matplotlib.pyplot as plt
def runAnalysis():
    # Parameters
    volt        = 3.5
    R_motor     = 0.15
    L_motor     = 0.0000006
    kv_motor    = 3216.11753
    kt_motor    = 0.001842
    dampConst   = 0.000001

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
            str(L_motor),
            str(kv_motor),
            str(kt_motor),
            str(dampConst)
        ]

        # Run the C++ binary
        result = subprocess.run(args, capture_output=True, text=True)

        # Print all lines
        lines = result.stdout.strip().splitlines()
        for line in lines:
            print(line)

        # Parse the last line
        try:
            last_line = lines[-1].strip()
            dcV, rpm, current, R_motor, kv_motor, kt_motor = map(float, last_line.split(','))
                
            print(f"Parsed values -> volt: {dcV}, rpm: {rpm}, current: {current}, L_motor: {L_motor}")

        except ValueError:
            print(f"Failed to parse last line: {last_line}")

        output_data.append({
            'volt': dcV,
            'rpm': rpm,
            'current': current,
            'R_motor': R_motor,
            'L_motor': L_motor,
            'kv_motor': kv_motor,
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

    plt.tight_layout()
    plt.show()


# runAnalysis()
plotData()