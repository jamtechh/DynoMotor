import subprocess
import csv

# Parameters
volt        = 3.5
R_motor     = 0.15
L_motor     = 0.00001
kv_motor    = 3216
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
            
        print(f"Parsed values -> volt: {dcV}, rpm: {rpm}, current: {current}")

    except ValueError:
        print(f"Failed to parse last line: {last_line}")

    output_data.append({
        'volt': dcV,
        'rpm': rpm,
        'current': current,
        'R_motor': R_motor,
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