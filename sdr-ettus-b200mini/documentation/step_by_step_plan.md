# Deploy Checklist — Step by Step

## Phase 1 — Verify Hardware
- [ ] Plug B210 into USB 3.0 port (blue port)
- [ ] Run `uhd_usrp_probe` and confirm it detects the device
- [ ] Check FPGA/FW versions match what we saw earlier
- [ ] Check USB is running at USB 3.0 (not 2.0) — `uhd_usrp_probe` will say "Operating over USB 3"
- [ ] free -h  -> What RAM does your DietPi board actually have?

---

## Phase 2 — Verify Dependencies
- [ ] Confirm nlohmann-json is installed: `dpkg -l | grep nlohmann`
- [ ] If not: `sudo apt install nlohmann-json3-dev`
- [ ] Confirm UHD headers are present: `ls /usr/include/uhd`

---

## Phase 3 — Compile & Run original `connect.cpp` (your first file, no threads)
- [ ] Make sure the original probe-style `connect.cpp` still compiles and runs clean
- [ ] This confirms your toolchain and USRP link are solid before adding complexity

---

## Phase 4 — Add JSON config, compile & run
- [ ] Add `params.json` and `config.hpp`
- [ ] Update `connect.cpp` to load config and print parameters
- [ ] Compile and run — confirm parameters print correctly
- [ ] **No RF yet**

---

## Phase 5 — Signal generation
- [ ] Add `signal.hpp` and `signal.cpp`
- [ ] Write a small standalone test that just prints `SIGNAL_TX.size()` and first/last sample
- [ ] Compile and confirm pulse length matches `FS × PULSE_DURATION`
- [ ] **No RF yet**

---

## Phase 6 — TX only, no antenna connected
- [ ] Add `tx_thread.hpp`
- [ ] Connect TX thread in `connect.cpp`
- [ ] **Leave TX/RX antenna port open (nothing connected)**
- [ ] Run for 1 second only (temporarily set `SESSION_DURATION: 1.0`)
- [ ] Confirm no UHD errors, pulse count prints correctly
- [ ] Watch for underruns `[U]` in terminal — means buffer starvation

---

## Phase 7 — GPIO test
- [ ] Add ATR GPIO config to `connect.cpp`
- [ ] Connect a multimeter or scope to FP0 pin 0 and GND
- [ ] Run TX-only session and confirm pin goes HIGH during bursts
- [ ] **Still no antenna**

---

## Phase 8 — SMA loopback, TX + RX
- [ ] Connect SMA cable: TX/RX port → RX2 port
- [ ] **Add 30dB attenuator in line** — protects RX from TX power
- [ ] Add `rx_thread.hpp` and connect RX thread
- [ ] Run short session (`SESSION_DURATION: 1.0`, `PRF: 100.0` to start slow)
- [ ] Confirm `rx_data_TIMESTAMP.bin` is created and has expected file size
- [ ] Check for overflows `[O]` in terminal

---

## Phase 9 — Validate data in Python
- [ ] Copy `.bin` file to your laptop
- [ ] Load and reshape:
```python
import numpy as np
import matplotlib.pyplot as plt

total_pulses = 275   # 100 Hz × 1s = 100, adjust
window_samps = 900   # (t_max - t_min) × FS

raw  = np.fromfile("rx_data.bin", dtype=np.int16)
IQ   = raw[0::2] + 1j * raw[1::2]
data = IQ.reshape(total_pulses, window_samps)

plt.figure()
plt.imshow(np.abs(data), aspect='auto')
plt.title("Range-time intensity")
plt.xlabel("Range sample")
plt.ylabel("Pulse")
plt.show()
```
- [ ] Confirm you see the TX pulse reflection in the range window

---

## Phase 10 — Full session
- [ ] Restore `SESSION_DURATION: 25.0` and `PRF: 2750.0`
- [ ] Confirm memory allocation prints expected MB
- [ ] Run full session
- [ ] Validate file size: `ls -lh rx_data_*.bin`

---

**Start with Phase 1 and tell me what you see at each step before moving on.**