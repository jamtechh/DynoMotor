import json
import matplotlib.pyplot as plt

# Load the JSON file
with open('output2.json', 'r') as file:
    data = json.load(file)

# Extract variables

torques = {
    "loadTorque": data.get("t_electronics", []),
    "Applied_Torque": data.get("T_motor", [])
}

t_mechanics = data.get("t_mechanics", [])
T_motor = data.get("T_motor", [])
angVel = data.get("dalpha", [])  # Separate angVel

# Define Y-axis variables (excluding angVel)
variables = {
    "Applied_Volt (V)": data.get("n1", []),
    "Back_EMF (V)": data.get("n3", []),
    "CurrentDrawn (A)": data.get("VmotorVAR", [])
}

# Find the minimum length to match all variables
min_len = min([len(t_mechanics)] + [len(v) for v in variables.values()] + [len(T_motor), len(angVel)])
t_mechanics = t_mechanics[:min_len]
T_motor = T_motor[:min_len]
angVel = angVel[:min_len]

# Truncate each variable to match the minimum length
for key in variables:
    variables[key] = variables[key][:min_len]
for key in torques:
    torques[key] = torques[key][:min_len]

# Create a 2x2 subplot grid
fig, axs = plt.subplots(2, 2, figsize=(14, 10))  # (rows, cols)

# ---- Plot 1: Multiple Variables vs t_mechanics (excluding angVel) ----
for key, values in variables.items():
    axs[0, 0].plot(t_mechanics, values, label=key, marker='o', markersize=3, linestyle='-')
axs[0, 0].set_xlabel('t_mechanics (Time)')
axs[0, 0].set_ylabel('Values')
axs[0, 0].set_title('Multiple Variables vs t_mechanics')
axs[0, 0].legend()
axs[0, 0].grid()

# ---- Plot 2: angVel vs t_mechanics ----
axs[0, 1].plot(t_mechanics, angVel, label="Angular Velocity (RPM)", marker='o', markersize=3, linestyle='-', color='g')
axs[0, 1].set_xlabel('t_mechanics (Time)')
axs[0, 1].set_ylabel('angVel')
axs[0, 1].set_title('angVel vs t_mechanics')
axs[0, 1].legend()
axs[0, 1].grid()

# ---- Plot 3: T_motor vs t_mechanics ----
for key, values in torques.items():
    axs[1, 0].plot(t_mechanics, values, label=key, marker='o', markersize=3, linestyle='-')
axs[1, 0].set_xlabel('t_mechanics (Time)')
axs[1, 0].set_ylabel('Torque kg-mm^2 / s^2')
axs[1, 0].set_title('Torque vs t_mechanics')
axs[1, 0].legend()
axs[1, 0].grid()

# Hide the unused subplot (bottom-right)
axs[1, 1].axis("off")

# Adjust layout for better spacing
plt.tight_layout()

# Show the plots
plt.show()
