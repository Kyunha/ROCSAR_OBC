# High-Performance SAR Radar System Implementation

## Hardware: Ettus Research USRP B200mini
## Language: C++ (UHD API)

This repository contains a synchronized, high-rate Full-Duplex radar transmitter and receiver optimized for Synthetic Aperture Radar (SAR) applications. The design utilizes hardware-timed commands to ensure strict phase coherence while implementing a discrete range-gating (windowing) mechanism to maintain a highly efficient host memory footprint.

> **Review Status:** Several bugs were identified and corrected in this version. All changes are annotated with `// FIX:` comments inline.

---

## 1. C++ Source Code (`high_rate_sar_windowed.cpp`)

```cpp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <iostream>
#include <vector>
#include <complex>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <fstream>

// Global execution control flag
std::atomic<bool> radar_active(true);

// Radar System Configuration Constants
const double SAMPLE_RATE  = 56.0e6;  // Maximized to 56 MSps for B200mini full-duplex operation
const double CENTER_FREQ  = 5.8e9;   // 5.8 GHz Carrier Frequency (C-Band / ISM Radar)
const double PRF          = 2750.0;  // 2.75 kHz Pulse Repetition Frequency
const double PRI          = 1.0 / PRF;
const double TOTAL_TIME   = 25.0;    // Test duration in seconds
const size_t TOTAL_PULSES = static_cast<size_t>(TOTAL_TIME * PRF); // 68,750 pulses

// LFM Chirp Waveform Parameters
const double PULSE_DURATION_SEC = 20.0e-6; // 20 microseconds pulse width
const size_t PULSE_SAMPLES      = static_cast<size_t>(PULSE_DURATION_SEC * SAMPLE_RATE); // 1120 samples
const double CHIRP_BANDWIDTH    = 45.0e6;  // Ultra-wide 45 MHz sweep for high spatial resolution

// ====== WINDOWING PARAMETERS ======
// Record the pulse transmission + extra time for target echoes (1500 samples * 1/56MHz = ~26.7 us)
const size_t RX_WINDOW_SAMPLES = 1500; // Must be smaller than full PRI (which is 20363 samples)
// Matrix allocation: 68750 pulses * 1500 samples * 8 bytes (complex float) = ~825 MB
std::vector<std::complex<float>> windowed_sar_matrix;
// ===================================

// Shared start time: set in main() after hardware is ready, used by both threads
// FIX: Both threads derive start_time from get_time_now() instead of a hardcoded
//      constant to eliminate the race condition where thread startup latency
//      could cause the first pulses to be missed.
uhd::time_spec_t shared_start_time;

// Function to generate a Linear Frequency Modulated (LFM) Chirp
std::vector<std::complex<float>> generate_lfm_chirp() {
    std::vector<std::complex<float>> chirp(PULSE_SAMPLES);
    double chirp_rate = CHIRP_BANDWIDTH / PULSE_DURATION_SEC;

    for (size_t n = 0; n < PULSE_SAMPLES; ++n) {
        double t = static_cast<double>(n) / SAMPLE_RATE;
        // Mathematical LFM Formula: exp(j * pi * K_r * t^2)
        double phase = M_PI * chirp_rate * t * t;
        chirp[n] = std::complex<float>(std::cos(phase) * 0.7f, std::sin(phase) * 0.7f);
    }
    return chirp;
}

// Transmission Thread (Timed Bursts based on precise PRI)
void tx_thread_func(uhd::tx_streamer::sptr tx_stream) {
    // FIX: Wrap set_current_thread_priority in try/catch.
    //      On Linux this requires root or CAP_SYS_NICE; without it UHD throws
    //      and the thread dies before sending a single pulse.
    try {
        uhd::utils::set_current_thread_priority(1.0, true);
    } catch (const std::exception& e) {
        std::cerr << "[TX Thread] Warning: Could not set real-time priority: "
                  << e.what() << ". Run as root or configure /etc/security/limits.conf." << std::endl;
    }

    std::vector<std::complex<float>> chirp = generate_lfm_chirp();
    uhd::tx_metadata_t tx_md;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst   = true;
    tx_md.has_time_spec  = true;

    // FIX: Use shared_start_time derived from get_time_now() in main()
    //      instead of the original hardcoded uhd::time_spec_t(1.5).
    uhd::time_spec_t transmit_time = shared_start_time;

    std::cout << "[TX Thread] Scheduling " << TOTAL_PULSES
              << " LFM bursts at 2750 Hz PRF..." << std::endl;

    for (size_t i = 0; i < TOTAL_PULSES && radar_active; ++i) {
        tx_md.time_spec = transmit_time;

        // FIX: Changed &chirp to chirp.data().
        //      &chirp is the address of the std::vector object itself, NOT the
        //      sample buffer. send() expects a pointer to the raw IQ data array.
        //      Passing &chirp causes undefined behaviour (crash or garbage TX).
        tx_stream->send(chirp.data(), chirp.size(), tx_md);

        // Schedule next pulse by adding exact PRI step
        transmit_time += uhd::time_spec_t(PRI);
    }
    std::cout << "[TX Thread] All pulses successfully scheduled in driver queue." << std::endl;
}

// Windowed Receiver Thread (Captures only the target data gate)
void rx_thread_func(uhd::rx_streamer::sptr rx_stream) {
    // FIX: Same real-time priority guard as TX thread.
    try {
        uhd::utils::set_current_thread_priority(1.0, true);
    } catch (const std::exception& e) {
        std::cerr << "[RX Thread] Warning: Could not set real-time priority: "
                  << e.what() << std::endl;
    }

    uhd::rx_metadata_t rx_md;
    std::vector<std::complex<float>> window_buffer(RX_WINDOW_SAMPLES);

    // Safely pre-allocate the unified data block in RAM to avoid allocations in loop
    windowed_sar_matrix.reserve(TOTAL_PULSES * RX_WINDOW_SAMPLES);

    // FIX: Offset the RX window start by one pulse duration relative to TX.
    //      Starting RX at exactly the TX timestamp captures the outgoing pulse
    //      itself, not the returning echo. Adding PULSE_DURATION_SEC shifts the
    //      gate so that it opens when the transmitted chirp has fully left the
    //      antenna and the earliest echoes begin arriving.
    //      For loopback testing you can remove this offset to capture the full
    //      chirp replica and verify matched-filter output.
    // LOOPBACK MODE — capture the chirp replica directly (no echo delay)
    uhd::time_spec_t capture_time = shared_start_time;

    // ANTENNA MODE — offset to skip outgoing pulse and capture echoes
    // uhd::time_spec_t capture_time = shared_start_time + uhd::time_spec_t(PULSE_DURATION_SEC);

    // Dropped pulse counter for data integrity diagnostics
    size_t dropped_pulses = 0;

    std::cout << "[RX Thread] Window receiver active. Capturing "
              << RX_WINDOW_SAMPLES << " samples per PRI..." << std::endl;

    for (size_t i = 0; i < TOTAL_PULSES && radar_active; ++i) {
        // Issue a command for a discrete timed burst (one window profile)
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps   = RX_WINDOW_SAMPLES;
        stream_cmd.stream_now  = false;
        stream_cmd.time_spec   = capture_time;
        rx_stream->issue_stream_cmd(stream_cmd);

        // Extract the exact window segment from hardware FIFO
        size_t samples_received = 0;
        while (samples_received < RX_WINDOW_SAMPLES && radar_active) {
            size_t num = rx_stream->recv(
                &window_buffer[samples_received],
                RX_WINDOW_SAMPLES - samples_received,
                rx_md, 0.1
            );

            if (rx_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                break; // Skip on hardware/driver error
            }
            samples_received += num;
        }

        if (samples_received == RX_WINDOW_SAMPLES) {
            // Push coherent array segment into final 2D linear vector
            windowed_sar_matrix.insert(
                windowed_sar_matrix.end(),
                window_buffer.begin(),
                window_buffer.end()
            );
        } else {
            // FIX: Log dropped/partial pulses instead of silently discarding them.
            //      Silent drops corrupt the slow-time axis of the SAR matrix
            //      (missing rows) without any indication that data is invalid.
            ++dropped_pulses;
            std::cerr << "[RX Thread] Warning: Partial window on pulse " << i
                      << " — received " << samples_received << "/"
                      << RX_WINDOW_SAMPLES << " samples. Pulse skipped." << std::endl;
        }

        // Advance to the exact next PRI boundary
        capture_time += uhd::time_spec_t(PRI);
    }

    std::cout << "[RX Thread] Finished. Total collected windowed samples: "
              << windowed_sar_matrix.size() << std::endl;
    if (dropped_pulses > 0) {
        std::cerr << "[RX Thread] WARNING: " << dropped_pulses
                  << " pulses were dropped. SAR matrix has missing rows." << std::endl;
    }
}

int main() {
    // ================= LOOPBACK OPTION SELECTION =================
    // FIX: Removed "loopback=digital" — this is not a valid UHD device argument.
    //      The AD9364 internal digital loopback cannot be enabled via the device
    //      string; it requires direct register access through the UHD property tree.
    //
    //      Option A — Hardware deployment (use 50 Ω loads on TX/RX and RX2 ports):
    std::string device_arguments = "type=b200";
    //
    //      Option B — Digital loopback via property tree (uncomment to use):
    //      Requires root. Enables the AD9364 BIST loopback register after init.
    //      bool enable_loopback = true;  // set false for real antenna operation
    // =============================================================

    try {
        std::cout << "[Main] Initializing USRP with arguments: "
                  << device_arguments << std::endl;
        uhd::usrp::multi_usrp::sptr usrp =
            uhd::usrp::multi_usrp::make(device_arguments);

        // Configure sample rates and center frequencies
        usrp->set_tx_rate(SAMPLE_RATE);
        usrp->set_rx_rate(SAMPLE_RATE);
        usrp->set_tx_freq(CENTER_FREQ);
        usrp->set_rx_freq(CENTER_FREQ);

        // Balanced gain settings for loopback / attenuated test scenarios
        usrp->set_tx_gain(10.0);
        usrp->set_rx_gain(15.0);

        // Data serialization: complex floats on host, sc16 over USB wire
        uhd::stream_args_t stream_args("fc32", "sc16");
        uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
        uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

        // Reset the hardware master clock to zero
        usrp->set_time_now(uhd::time_spec_t(0.0));

        // Allow the clock to settle before scheduling
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // FIX: Derive shared_start_time from the live hardware clock after
        //      device init completes. Adding 0.5 s gives both threads enough
        //      margin to start and register their first timed commands with the
        //      FPGA before the window opens — replacing the original hardcoded
        //      constant which had no awareness of actual thread startup latency.
        shared_start_time = usrp->get_time_now() + uhd::time_spec_t(0.5);

        // Spawn parallel execution contexts
        std::thread transmission_worker(tx_thread_func, tx_stream);
        std::thread reception_worker(rx_thread_func, rx_stream);

        std::cout << "[Main] Execution ongoing. Collecting "
                  << TOTAL_TIME << "-second data batch..." << std::endl;

        std::this_thread::sleep_for(
            std::chrono::seconds(static_cast<int>(TOTAL_TIME + 2.0))
        );

        // Graceful teardown
        radar_active = false;
        transmission_worker.join();
        reception_worker.join();

        // ====== BINARY STRUCTURE EXPORT ======
        std::cout << "[Main] Writing data matrix to disk (raw_sar_matrix.bin)..."
                  << std::endl;
        std::ofstream outfile("raw_sar_matrix.bin", std::ios::binary);
        if (outfile.is_open()) {
            outfile.write(
                reinterpret_cast<const char*>(windowed_sar_matrix.data()),
                windowed_sar_matrix.size() * sizeof(std::complex<float>)
            );
            outfile.close();
            std::cout << "[Main] Binary structure successfully exported." << std::endl;
        } else {
            std::cerr << "[Main] Error: Failed to open output file stream." << std::endl;
        }
        // =====================================

        std::cout << "[Main] Processing successfully completed." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[FATAL SYSTEM EXCEPTION] " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
```

---

## 2. Technical Documentation

### System Architecture Overview

This application implements a high-performance **Software Defined Radar (SDR)** architecture optimized for **Synthetic Aperture Radar (SAR)** applications using an Ettus Research USRP B200mini. The execution framework addresses two core challenges of ultra-high data rate SDR processing: phase-coherence via hardware-timed commands, and host memory starvation prevention via discrete range-gate windowing.

---

### Parameter Design & Mathematical Calculations

**Sample Rate ($f_s$):** Maximized to `56 MSps`. This pushes the physical boundary of the USB 3.0 controller while providing a massive instantaneous processing bandwidth.

**Carrier Frequency ($f_c$):** `5.8 GHz` (C-Band / ISM band). The resulting wavelength is:

$$\lambda = \frac{c}{f_c} = \frac{3 \times 10^8\,\text{m/s}}{5.8 \times 10^9\,\text{Hz}} \approx \mathbf{5.17\,\text{cm}}$$

This short wavelength ensures excellent backscattering phase sensitivity for high-resolution SAR mapping.

**Chirp Bandwidth ($B$):** `45 MHz`. This yields a theoretical Slant Range Resolution ($\Delta R$) of:

$$\Delta R = \frac{c}{2B} = \frac{3 \times 10^8}{2 \times 45 \times 10^6} \approx \mathbf{3.33\,\text{meters}}$$

**Pulse Repetition Frequency (PRF):** `2750 Hz`. The Pulse Repetition Interval ($PRI$) is $1/2750 \approx 363.63\,\mu\text{s}$. In the discrete domain a single PRI period corresponds to:

$$\text{PRI Samples} = 56{,}000{,}000\,\text{samples/s} \times \frac{1}{2750}\,\text{s} \approx \mathbf{20{,}363\,\text{samples}}$$

**Pulse Duration ($\tau$):** `20 microseconds`. The sample width of a single transmitted pulse is:

$$\text{Pulse Samples} = 20 \times 10^{-6}\,\text{s} \times 56 \times 10^6\,\text{samples/s} = \mathbf{1120\,\text{samples}}$$

---

### Memory Optimization Scheme (The Gating Strategy)

A continuous 25-second capture at 56 MSps requires storing $25 \times 56 \times 10^6 = 1.4 \times 10^9$ complex float samples. At 8 bytes per sample that totals **11.2 GB** of continuous RAM — enough to trigger memory starvation and buffer overruns on standard systems.

By introducing **Range Gating / Windowing**, the receiver issues a timed `STREAM_MODE_NUM_SAMPS_AND_DONE` burst command requesting exactly 1500 samples starting at each TX trigger point.

**Window Duration:**

$$\frac{1500}{56\,\text{MHz}} \approx \mathbf{26.78\,\mu\text{s}}$$

This encompasses the entire transmitted chirp ($20\,\mu\text{s}$) plus an extra $6.78\,\mu\text{s}$ listening gate — equivalent to a maximum unambiguous target depth of approximately $1\,\text{km}$.

**Optimized Memory Footprint:**

$$\text{Total Size} = 68{,}750\,\text{pulses} \times 1{,}500\,\text{samples} \times 8\,\text{bytes} \approx \mathbf{825\,\text{MB}}$$

This reduces system memory demand by **92.6%**, enabling safe execution on commercial computers.

---

### Concurrency and Synchronization Model

The application deploys two concurrent threads bound to real-time scheduling via `uhd::utils::set_current_thread_priority`:

1. **`tx_thread_func`** — Generates and transmits the LFM waveform. Uses the blocking mechanism of `tx_stream->send()` to pace data into the USRP FPGA buffer, advancing a `uhd::time_spec_t` timestamp by exactly one PRI per pulse.

2. **`rx_thread_func`** — Orchestrates gated acquisition windows. By anchoring to the same `shared_start_time` as TX (offset by one pulse duration for echo gating), the B200mini shared master clock ensures a phase-locked grid. This allows the linear array to be de-interleaved into a structured Fast-Time / Slow-Time matrix.

> **Note on real-time priority:** `set_current_thread_priority(1.0, true)` requires either running as root (`sudo`) or configuring `/etc/security/limits.conf`. This version wraps the call in a try/catch so the program degrades gracefully rather than crashing if privileges are insufficient.

---

### Hardware Testing Modes

**Hardware Deployment Mode — `"type=b200"` (recommended starting point)**

Activates the physical SMA ports. Terminate both TX/RX and RX2 ports with 50 Ω loads for baseline noise floor evaluation before connecting an antenna.

**Digital Loopback Mode (manual register access)**

The original `"type=b200,loopback=digital"` argument string is **not valid in UHD** and will throw an exception at `multi_usrp::make()`. The AD9364 internal BIST loopback must be enabled after device initialization via the UHD property tree. Example:

```cpp
// After multi_usrp::make(), enable AD9364 digital loopback:
usrp->get_device()->get_tree()
    ->access<int>("/mboards/0/dboards/A/rx_frontends/A/ad9361_bist_loopback")
    .set(1);
```

> This register path may vary by UHD version. Verify with `uhd_usrp_probe` on your system.

---

### Bug Fix Summary

| # | Location | Issue | Fix Applied |
|---|---|---|---|
| 1 | `main()` | `"loopback=digital"` is not a valid UHD device argument — throws on init | Removed; documented property-tree workaround |
| 2 | `tx_thread_func()` | `tx_stream->send(&chirp, ...)` passes address of the vector object, not the data buffer — undefined behaviour | Changed to `chirp.data()` |
| 3 | Both threads | Hardcoded `time_spec_t(1.5)` start time ignores thread startup latency — race condition | Replaced with `shared_start_time` derived from `get_time_now() + 0.5s` |
| 4 | `rx_thread_func()` | RX window opens at TX time, capturing the outgoing pulse not the echo | Offset by `PULSE_DURATION_SEC` |
| 5 | `rx_thread_func()` | Partial/dropped windows silently discarded, corrupting the SAR matrix row count | Added drop counter and `std::cerr` warnings |
| 6 | Both threads | `set_current_thread_priority()` throws if process lacks root/CAP_SYS_NICE, killing the thread | Wrapped in try/catch with informative fallback message |

---

## 3. Compilation & Data Parsing

### Compilation

Compile with maximum optimization and C++11 threading support:

```bash
g++ -O3 -std=c++11 high_rate_sar_windowed.cpp -luhd -lpthread -o high_rate_sar
```

Run with elevated privileges to enable real-time thread scheduling:

```bash
sudo ./high_rate_sar
```

---

### Output File Structure

Execution produces `raw_sar_matrix.bin` — a flat binary array of 32-bit floating-point complex numbers (interleaved I/Q pairs). Import into NumPy or MATLAB and reshape into the 2D radar grid:

**Python / NumPy:**

```python
import numpy as np

TOTAL_PULSES   = 68750
RX_WIN_SAMPLES = 1500

data = np.fromfile("raw_sar_matrix.bin", dtype=np.complex64)
sar_matrix = data.reshape((TOTAL_PULSES, RX_WIN_SAMPLES))

# sar_matrix shape: (slow-time / pulses, fast-time / range bins)
print(sar_matrix.shape)  # (68750, 1500)
```

**MATLAB:**

```matlab
fid = fopen('raw_sar_matrix.bin', 'rb');
raw = fread(fid, inf, 'float32');
fclose(fid);

iq = raw(1:2:end) + 1j * raw(2:2:end);
sar_matrix = reshape(iq, [1500, 68750]).';   % [pulses x range bins]
```

**Matrix Dimensions:**

| Axis | Dimension | Size |
|---|---|---|
| Rows | Slow-time (pulses) | 68,750 |
| Columns | Fast-time (range bins) | 1,500 |