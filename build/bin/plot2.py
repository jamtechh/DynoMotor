import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import interp1d

# Function to extract the upper envelope using peak detection and interpolation
def upper_envelope(data, t_data):
    peaks, _ = find_peaks(data)
    if len(peaks) < 2:  # Ensure enough peaks for interpolation
        return data  # Return original if not enough peaks
    peak_times = t_data[peaks]
    peak_values = data[peaks]
    interp_func = interp1d(peak_times, peak_values, kind='linear', fill_value="extrapolate")
    return interp_func(t_data)

# Function to extract the lower envelope using trough detection and interpolation
def lower_envelope(data, t_data):
    troughs, _ = find_peaks(-data)  # Invert data to find troughs
    if len(troughs) < 2:
        return data
    trough_times = t_data[troughs]
    trough_values = data[troughs]
    interp_func = interp1d(trough_times, trough_values, kind='linear', fill_value="extrapolate")
    return interp_func(t_data)

# Load JSON data from file
def load_data(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

# Main function to process and plot the data
def process_and_plot(file_path):
    # Load the JSON file
    data = load_data(file_path)

    # Extract variables
    time = np.array(data.get("t_mechanics", []))
    loadTorque = np.array(data.get("t_electronics", []))
    T_motor = np.array(data.get("T_motor", []))
    angVel = np.array(data.get("dalpha", []))  # Keep angVel as it is
    n3 = np.array(data.get("n3", []))
    VmotorVAR = np.array(data.get("VmotorVAR", []))

    # Ensure all arrays have the same length
    min_len = min(len(time), len(T_motor), len(angVel), len(n3), len(VmotorVAR))
    time = time[:min_len]
    T_motor = T_motor[:min_len]
    angVel = angVel[:min_len]
    n3 = n3[:min_len]
    VmotorVAR = VmotorVAR[:min_len]
    loadTorque = loadTorque[:min_len]

    # Compute envelopes
    # T_motor_upper = upper_envelope(T_motor, time)  # Upper boundary for T_motor
    # n3_lower = lower_envelope(n3, time)  # Lower boundary for n3
    # VmotorVAR_lower = lower_envelope(VmotorVAR, time)  # Lower boundary for VmotorVAR

    # Create a 2x2 subplot grid
    fig, axs = plt.subplots(2, 1, figsize=(14, 10))  # (rows, cols)

    # ---- Plot 1: Multiple Variables vs time ----
    # axs[0, 0].plot(time, n3_lower, label="n3 (Lower)", linestyle='-', color='orange')
    # axs[0, 0].plot(time, VmotorVAR_lower, label="VmotorVAR (Lower)", linestyle='-', color='green')
    # axs[0, 0].set_xlabel('time (Time)')
    # axs[0, 0].set_ylabel('Values')
    # axs[0, 0].set_title('Multiple Variables vs time')
    # axs[0, 0].legend()
    # axs[0, 0].grid()

    # ---- Plot 2: angVel vs time ----
    axs[0].plot(time, angVel, label="angVel", linestyle='-', color='g')
    axs[0].set_xlabel('time (Time)')
    axs[0].set_ylabel('angVel')
    axs[0].set_title('angVel vs time')
    axs[0].legend()
    axs[0].grid()
    # ---lot 3: T_motor vs time ----
    axs[1].plot(time, T_motor, label="applied torque", linestyle='-', color='r')
    axs[1].plot(time, loadTorque, label="Load", linestyle='-', color='g')
    axs[1].set_xlabel('time (Time)')
    axs[1].set_ylabel('T_motor (N-mm)')
    axs[1].set_title('T_motor vs time')
    axs[1].legend()
    axs[1].grid()

    # Hide the unused subplot (bottom-right)
    # axs[1, 1].axis("off")

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()

# Run the script with the provided JSON file
if __name__ == "__main__":
    file_path = "output2.json"  # Update with your file path
    process_and_plot(file_path)
