Here's a complete guide to connecting a C++ program to the **Ettus B210 mini** on a **Raspberry Pi 4** using the UHD (USRP Hardware Driver) library.

---

## Overview

The Ettus B210 is a Software Defined Radio (SDR) device that connects via USB 3.0. Ettus's official C++ API is **UHD (USRP Hardware Driver)**. Here's the full path:

```
Raspberry Pi 4 (USB 3.0)
       ↓
   Ettus B210 Mini
       ↓
   UHD Library (C++ API)
       ↓
   Your C++ Program
```

---

## Step 1: Install UHD on Raspberry Pi OS

```bash
sudo apt update
sudo apt install -y libuhd-dev uhd-host cmake g++ build-essential
```

Then download the UHD FPGA images (required for the B210):

```bash
sudo uhd_images_downloader
```

---

## Step 2: Verify the B210 is detected

Plug in the B210 via USB 3.0, then run:

```bash
uhd_find_devices
```

You should see something like:
```
[INFO] [UHD] linux; GNU C++ version ...
--------------------------------------------------
-- UHD Device 0
--------------------------------------------------
Device Address:
    serial: XXXXXXXX
    name: MyB210
    product: B210
    type: b200
```

If nothing shows up, run `uhd_usrp_probe` to debug.

---

## Step 3: A minimal C++ program

```cpp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/types/tune_request.hpp>
#include <iostream>
#include <complex>
#include <vector>

int UHD_SAFE_MAIN(int argc, char* argv[]) {

    // 1. Create USRP device handle (empty string = auto-detect)
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make("");

    std::cout << "Using device: " << usrp->get_pp_string() << std::endl;

    // 2. Configure RX channel
    double sample_rate = 1e6;     // 1 MSPS
    double center_freq = 100e6;   // 100 MHz (FM band)
    double gain        = 30.0;    // dB
    double bandwidth   = 1e6;     // 1 MHz

    usrp->set_rx_rate(sample_rate);
    usrp->set_rx_freq(uhd::tune_request_t(center_freq));
    usrp->set_rx_gain(gain);
    usrp->set_rx_bandwidth(bandwidth);

    // 3. Create RX stream
    uhd::stream_args_t stream_args("fc32"); // 32-bit complex float
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // 4. Start streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    size_t num_samples = 1000;
    stream_cmd.num_samps  = num_samples;
    stream_cmd.stream_now = true;
    rx_stream->issue_stream_cmd(stream_cmd);

    // 5. Receive samples
    std::vector<std::complex<float>> buffer(num_samples);
    uhd::rx_metadata_t md;

    size_t received = rx_stream->recv(&buffer.front(), buffer.size(), md, 3.0);

    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        std::cerr << "Receive error: " << md.strerror() << std::endl;
        return 1;
    }

    std::cout << "Received " << received << " samples." << std::endl;

    // 6. Print first 5 samples
    for (size_t i = 0; i < 5 && i < received; i++) {
        std::cout << "  sample[" << i << "] = "
                  << buffer[i].real() << " + "
                  << buffer[i].imag() << "j" << std::endl;
    }

    return 0;
}
```

---

## Step 4: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(b210_example)

set(CMAKE_CXX_STANDARD 17)

find_package(UHD REQUIRED)

add_executable(b210_example main.cpp)

target_include_directories(b210_example PRIVATE ${UHD_INCLUDE_DIRS})
target_link_libraries(b210_example PRIVATE ${UHD_LIBRARIES})
```

---

## Step 5: Build and run

```bash
mkdir build && cd build
cmake ..
make -j4
./b210_example
```

---

## Key UHD Concepts

| Concept | Description |
|---|---|
| `multi_usrp::make("")` | Auto-detects any connected USRP |
| `set_rx_rate()` | Sample rate (B210 supports ~200 kSPS–56 MSPS) |
| `set_rx_freq()` | Center frequency (B210: 70 MHz – 6 GHz) |
| `set_rx_gain()` | Gain in dB (0–76 dB on B210) |
| `fc32` | 32-bit complex float samples (most common) |
| `sc16` | 16-bit complex int (lower CPU usage, good for Pi 4) |

---

## Tips for Raspberry Pi 4

- **Use USB 3.0** — plug into the blue ports. USB 2.0 won't sustain high sample rates.
- **Lower your sample rate** initially (1–5 MSPS) to avoid overflows on the Pi 4.
- Watch for `O` (overflow) characters in logs — reduce sample rate if seen.
- Consider `sc16` instead of `fc32` to halve the data bandwidth.
- Set real-time priority: `uhd::set_thread_priority_safe()` at the top of `main`.



#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <iostream>
#include <stdexcept>

int UHD_SAFE_MAIN(int argc, char* argv[]) {

    uhd::set_thread_priority_safe();

    uhd::usrp::multi_usrp::sptr usrp;

    try {
        std::cout << "Connecting to B210..." << std::endl;
        usrp = uhd::usrp::multi_usrp::make(std::string(""));
    }
    catch (const uhd::exception& e) {
        std::cerr << "UHD error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Connected!" << std::endl;
    std::cout << "  Board   : " << usrp->get_mboard_name()   << std::endl;
    std::cout << "  RX Rate : " << usrp->get_rx_rate() / 1e6 << " MSPS" << std::endl;
    std::cout << "  TX Rate : " << usrp->get_tx_rate() / 1e6 << " MSPS" << std::endl;
    std::cout << "  RX Freq : " << usrp->get_rx_freq() / 1e6 << " MHz"  << std::endl;
    std::cout << "  RX Gain : " << usrp->get_rx_gain()        << " dB"   << std::endl;


    // Disconnect
    usrp.reset();
    std::cout << "Disconnected." << std::endl;

    return 0;
}

