import copy
import math
import dataAnalysis2.py
best_error = float('inf')
best_params = {}
errors = []

original_kt = kt_motor
original_ke = ke_motor

# Small variations (tune as needed)
kt_range = [original_kt * (1 + 0.01 * i) for i in range(-5, 6)]  # -5% to +5%
ke_range = [original_ke * (1 + 0.01 * i) for i in range(-5, 6)]  # -5% to +5%

iteration = 0
for new_kt in kt_range:
    for new_ke in ke_range:
        iteration += 1
        print(f"\n=== Iteration {iteration} | kt: {new_kt:.7f}, ke: {new_ke:.7f} ===")

        # Set the globals
        kt_motor = new_kt
        ke_motor = new_ke

        # Run the analysis
        runAnalysis()

        # Load data to compute error
        try:
            with open("analysis/noLoad2.csv", newline='') as f:
                reader = csv.DictReader(f)
                input_data = [row for row in reader]
            with open("analysis/motor_output.csv", newline='') as f:
                reader = csv.DictReader(f)
                sim_data = [row for row in reader]
        except Exception as e:
            print(f"Error reading CSV: {e}")
            continue

        total_error = 0.0
        count = min(len(input_data), len(sim_data))
        for i in range(count):
            try:
                input_rpm = float(input_data[i]['rpm'])
                input_current = float(input_data[i]['current'])

                sim_rpm = float(sim_data[i]['rpm'])
                sim_current = float(sim_data[i]['current'])

                # Compute simple absolute error
                rpm_err = abs(input_rpm - sim_rpm)
                curr_err = abs(input_current - sim_current)

                total_error += (rpm_err + curr_err) / 2.0

            except Exception as e:
                print(f"Parse error on row {i}: {e}")

        avg_error = total_error / count
        errors.append((avg_error, new_kt, new_ke))
        print(f"Average error: {avg_error:.5f}")

        if avg_error < best_error:
            best_error = avg_error
            best_params = {'kt_motor': new_kt, 'ke_motor': new_ke}

print("\n=== DONE ===")
print(f"Best avg error: {best_error:.5f}")
print(f"Best parameters: kt_motor = {best_params['kt_motor']:.7f}, ke_motor = {best_params['ke_motor']:.7f}")