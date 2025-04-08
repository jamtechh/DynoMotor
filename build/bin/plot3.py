import json
import matplotlib.pyplot as plt

import os
import re

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

# Run the latest file
if latest_file:
    print(f"Running latest file: {latest_file}")
else:
    print("No matching files found.")

# Load the JSON file
with open("outputPlot/"+latest_file, 'r') as file:
    data = json.load(file)

# Extract variables
t_mechanics = data.get("t_mechanics", [])
T_motor = data.get("T_motor", [])
angVel = data.get("angVel", []) 
angVel_data = data.get("angVel_data", []) 
loadTorque = data.get("loadTorque", [])  # Separate angVel

# Define Y-axis variables (excluding angVel)
variables = {
    # "Applied_Volt (V)": data.get("Applied_Volt", []),
    "Back_EMF (V)": data.get("Back_EMF", [])
    # "Current (A)": data.get("Current", [])
    # "Angle": data.get("alpha", [])
    # "t_electronics": data.get("t_electronics", [])
}

# Find the minimum length to match all variables
min_len = min([len(t_mechanics)] + [len(v) for v in variables.values()] + [len(T_motor), len(angVel)])
t_mechanics = t_mechanics[:min_len]
T_motor = T_motor[:min_len]
angVel = angVel[:min_len]
angVel_data = angVel_data[:min_len]
loadTorque = loadTorque[:min_len]

# Truncate each variable to match the minimum length
for key in variables:
    variables[key] = variables[key][:min_len]

# Create a 2x2 subplot grid
fig, axs = plt.subplots(2, 2, figsize=(14, 10))  # (rows, cols)

# ---- Plot 1: Multiple Variables vs t_mechanics (excluding angVel) ----
for key, values in variables.items():
    axs[0, 1].plot(t_mechanics, values, label=key, linestyle='-')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Values')
axs[0, 1].set_title('Electrical Variables vs time')
axs[0, 1].legend()
axs[0, 1].grid()

# ---- Plot 2: angVel vs t_mechanics ----
axs[0, 0].plot(t_mechanics, angVel, label="RPM_simulation", linestyle='-', color='g')
axs[0, 0].plot(t_mechanics, angVel_data, label="RPM_data", linestyle='-', color='r')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('RPM')
axs[0, 0].set_title('Angular Velocity vs time')
axs[0, 0].legend()
axs[0, 0].grid()

# ---- Plot 1: T_motor vs t_mechanics ----
# axs[1, 0].plot(t_mechanics, T_motor, label="Applied Torque", linestyle='-', color='r')
axs[1, 0].plot(t_mechanics, loadTorque, label="load Torque", linestyle='-', color='g')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Torque(N-mm)')
axs[1, 0].set_title('Applied Torque vs time')
axs[1, 0].legend()
axs[1, 0].grid()

# Hide the unused subplot (bottom-right)
axs[1, 1].axis("off")

# Adjust layout for better spacing
plt.tight_layout()

# Show the plots
plt.show()
