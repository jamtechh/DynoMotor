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

# Load the JSON file
jasonFile = "outputPlot/"+latest_file
# jasonFile = "outputPlot/run_13_output_10000_9e-06.json"
print(f"Running latest file: {jasonFile}")
with open(jasonFile, 'r') as file:
    data = json.load(file)

# Extract variables
t_mechanics = data.get("t_mechanics", [])
T_motor = data.get("T_motor", [])
angVel = data.get("angVel", []) 
angVel_data = data.get("angVel_data", []) 
loadTorque = data.get("loadTorque", [])  # Separate angVel

# Define Y-axis variables (excluding angVel)
variables = {
    "Applied_Volt (V)": data.get("Applied_Volt", []),
    "Back_EMF (V)": data.get("Back_EMF", []),
    # "PWM (DutyCycle)": data.get("PWM", []),
    "Current (A)": data.get("Current", [])
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

# # Create a 2x2 subplot grid
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
axs[0, 0].set_xlabel('Time (milli seconds)')
axs[0, 0].set_ylabel('RPM')
axs[0, 0].set_title('Angular Velocity vs time')
axs[0, 0].legend()
axs[0, 0].grid()

# ---- Plot 1: T_motor vs t_mechanics ----
axs[1, 0].plot(t_mechanics, T_motor, label="Applied Torque", linestyle='-', color='r')
axs[1, 0].plot(t_mechanics, loadTorque, label="load Torque", linestyle='-', color='g')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Torque(Kg-mm^2 / s^2)')
axs[1, 0].set_title('Applied Torque vs time')
axs[1, 0].legend()
axs[1, 0].grid()

# Hide the unused subplot (bottom-right)
axs[1, 1].axis("off")

max_current = max(variables['Current (A)'])
max_rpm = max(angVel)
max_index = angVel.index(max_rpm)
max_time = t_mechanics[max_index]
last_time = max(t_mechanics)
last_index = t_mechanics.index(last_time)
last_torque = T_motor[last_index]
last_current = variables['Current (A)'][last_index]
# Annotate it

meta = data.get("meta", {})
timestep = meta.get("timestep")
kv_motor = meta.get("kv_motor")
kt_motor = meta.get("kt_motor")
ke_motor = meta.get("ke_motor")
Resistance = meta.get("Resistance")
Inductance = meta.get("Inductance")
dampConst = meta.get("dampConst")
AppliedVolt = meta.get("AppliedVolt")
# sim_name = meta.get("sim_name")
version = meta.get("version")

info = ''
info += f"max_current: {max_current:.3f} A\n"
info += f"max_rpm: {max_rpm:.3f}\n"
info += f"steady_torque: {last_torque:.3f}\n"
info += f"steady_current: {last_current:.3f}\n"

info += "\n\nSimulation Parametes:\n"
info += f"timeStep = {timestep} s\n"
info += f"AppliedVolt: {AppliedVolt:.3f} V\n"
info += f"kv_motor: {kv_motor:.3f} Rpm/V\n"
info += f"kt_motor = {kt_motor:.3f} Nm/A\n"
info += f"ke_motor: {ke_motor:.3f} V/(rad/s)\n"
info += f"Resistance: {Resistance:.3f} Ohms\n"
info += f"Inductance = {Inductance} H\n"
info += f"damping_factor = {dampConst}\n"

fig.text(
    0.7, 0.02,  # X, Y position (from 0 to 1, figure coords)
    info,  # Text to display
    ha='left', va='bottom',  # Align text to corner
    fontsize=15, color='black'
)
# Adjust layout for better spacing
plt.tight_layout()


# Create the plot
# plt.figure(figsize=(8, 5))
# # for key, values in variables.items():
# #     plt.plot(t_mechanics, values, label=key, linestyle='-')
# plt.plot(t_mechanics, angVel, label="RPM_motor", linestyle='-', color='g')
# # Add labels and title
# plt.xlabel('Time (s)')
# plt.ylabel("RPM")
# plt.title("Simulation Timestep = 1e-5")
# plt.grid(True)
# plt.legend()

imageDir = "outputImage/Image_" + str(latest_run) + ".png"
plt.savefig(imageDir, dpi=300, bbox_inches='tight')

# Show the plots
plt.show()
