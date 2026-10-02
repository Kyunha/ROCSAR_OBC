import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal
import time
import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent.parent

# Read raw int16 interleaved IQ binary file
tx_file_path = "../sdr-ettus-b200mini/Data/tx_chirp_only.bin"
raw_tx = np.fromfile(tx_file_path, dtype=np.int16)

# Convert to complex NumPy signal
tx_chirp = raw_tx[0::2] + 1j * raw_tx[1::2]

# --- Load Configuration from JSON ---
with open(REPO_DIR / "sdr-ettus-b200mini" / "parameters" / "params.json", "r") as f:
    config = json.load(f)

# --- 1. RX Data Parsing ---
PRF          = config["PRF"]
FS           = config["FS"]
T_MIN_US     = config["T_MIN_US"]
T_MAX_US     = config["T_MAX_US"]

window_samps = int((T_MAX_US - T_MIN_US) * 1e-6 * FS)

file_path = Path(sys.argv[1]) if len(sys.argv) > 1 else SCRIPT_DIR / "rx_data_20260722_154348_25s_0_200us.bin"
raw = np.fromfile(file_path, dtype=np.int16)
# get bin name
bin_name = os.path.basename(file_path)
# just get the rx_data_20260722_154348_25s_0_200us part
bin_name = bin_name.split(".")[0]
IQ = raw[0::2] + 1j * raw[1::2]

actual_pulses = len(IQ) // window_samps
valid_samples = actual_pulses * window_samps
data = IQ[:valid_samples].reshape(actual_pulses, window_samps)

def matched_filter(rx, template):
    """
    Cross-correlate rx against template and return an array the SAME
    LENGTH as rx, where output[n] is the correlation value corresponding
    to the template match STARTING at rx[n].

    This replaces scipy.signal.correlate(..., mode='same'), which instead
    centers the output on the template and introduces a
    +len(template)/2 sample bias in the reported peak location.

    Implementation: run mode='full' (length len(rx)+len(template)-1) and
    slice starting at index (len(template)-1), which is exactly where a
    match starting at rx[0] appears in the full correlation.
    """
    full = signal.correlate(rx, template, mode='full', method='fft')
    start = len(template) - 1
    return full[start:start + len(rx)]


# --- 3. Matched Filtering (2D Cross-Correlation) ---
print(f"Applying matched filter to all {actual_pulses} pulses...")
compressed_data = np.zeros_like(data, dtype=np.complex128)

for i in range(actual_pulses):
    compressed_data[i, :] = matched_filter(data[i, :], tx_chirp)

# --- 4. Save compressed data as a single headless PNG ---
session_duration = actual_pulses / PRF

fig_rti = plt.figure(figsize=(12, 8))
plt.imshow(np.abs(compressed_data),
           aspect='auto',
           origin='lower',
           extent=[T_MIN_US, T_MAX_US, 0, session_duration],
           cmap='viridis')

plt.colorbar(label='Correlation Magnitude')
plt.xlabel('Time within RX Window [µs] (Fast Time / Range)')
plt.ylabel('Elapsed Time [s] (Slow Time)')
plt.title(f'Range-Time Intensity Image ({actual_pulses} pulses)')
plt.grid(False)
plt.tight_layout()

# dont pause on plt.show() so that the script can continue to save the plots


# save all as png to a folder with timestamp and file name
save_folder = REPO_DIR / "plots" / f"matched_filter_{time.strftime('%Y%m%d_%H%M%S')}_{bin_name}"
os.makedirs(save_folder, exist_ok=True)
fig_rti.savefig(save_folder / "matched_filter_output.png", dpi=120)
plt.close(fig_rti)

print(f"Compressed data PNG saved to: {save_folder / 'matched_filter_output.png'}")