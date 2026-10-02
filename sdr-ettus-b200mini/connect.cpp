#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <vector>
#include <complex>

#include <sys/mman.h>
#include <sched.h>
#include <pthread.h>
#include <errno.h>
#include <cstring>

#include "parameters/config.hpp"
#include "libs/signal/signal.hpp"
#include "libs/tx/tx.hpp"
#include "libs/rx/rx.hpp"



void configure_realtime()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::cerr << "[RT] mlockall failed: " << strerror(errno) << "\n";

    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        std::cerr << "[RT] sched_setscheduler failed: " << strerror(errno) << "\n";

    std::cout << "[RT] Real-time scheduler active, priority: "
              << sp.sched_priority << "\n";
}

void pin_thread(std::thread& t, int core)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_setaffinity_np(t.native_handle(), sizeof(cpuset), &cpuset) != 0)
        std::cerr << "[RT] Failed to pin thread to core " << core << "\n";
    else
        std::cout << "[RT] Thread pinned to core " << core << "\n";
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    configure_realtime();  // first thing, before USRP init

    uhd::set_thread_priority_safe();

    // --- Load config ---
    const Config cfg = load_config("./../sdr-ettus-b200mini/parameters/params.json");

    // --- Connect ---
    uhd::usrp::multi_usrp::sptr usrp;
    try {
        std::cout << "Connecting to B210..." << std::endl;
        usrp = uhd::usrp::multi_usrp::make(std::string(""));
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // --- Configure TX ---
    usrp->set_tx_rate(cfg.fs);
    usrp->set_tx_freq(uhd::tune_request_t(cfg.tx_freq));
    usrp->set_normalized_tx_gain(cfg.normalized_gain_tx);
    usrp->set_tx_bandwidth(cfg.fs);
    usrp->set_tx_antenna(cfg.tx_antenna, 0);  // channel 0

    // --- Configure GPIO for TX indicator (ATR hardware mode) ---
    // usrp->set_gpio_attr("FP0", "CTRL",   0x0001, 0x0001); // pin 0 = ATR controlled
    // usrp->set_gpio_attr("FP0", "DDR",    0x0001, 0x0001); // pin 0 = output
    // usrp->set_gpio_attr("FP0", "ATR_TX", 0x0001, 0x0001); // HIGH during TX burst
    // usrp->set_gpio_attr("FP0", "ATR_0X", 0x0000, 0x0001); // LOW when idle
    // std::cout << "[GPIO] TX indicator configured (FP0 pin 0, ATR mode)\n";

    // --- Configure RX ---
    usrp->set_rx_rate(cfg.fs);
    usrp->set_rx_freq(uhd::tune_request_t(cfg.tx_freq));  // monostatic
    usrp->set_normalized_rx_gain(cfg.normalized_gain_rx);
    usrp->set_rx_bandwidth(cfg.fs);
    usrp->set_rx_antenna(cfg.rx_antenna, 0);  // channel 0

    std::cout << "  Board      : " << usrp->get_mboard_name()          << "\n"
            << "  TX Rate    : " << usrp->get_tx_rate() / 1e6        << " MSPS\n"
            << "  TX Freq    : " << usrp->get_tx_freq() / 1e6        << " MHz\n"
            << "  TX Gain    : " << usrp->get_normalized_tx_gain()   << " (normalized)\n"
            << "  TX Antenna : " << usrp->get_tx_antenna()           << "\n"
            << "  RX Rate    : " << usrp->get_rx_rate() / 1e6        << " MSPS\n"
            << "  RX Freq    : " << usrp->get_rx_freq() / 1e6        << " MHz\n"
            << "  RX Gain    : " << usrp->get_normalized_rx_gain()   << " (normalized)\n"
            << "  RX Antenna : " << usrp->get_rx_antenna()           << "\n";

    // Verify monostatic configuration
    if (usrp->get_tx_antenna() == usrp->get_rx_antenna()) {
        std::cout << "[Config] Monostatic mode ACTIVE on port: " << usrp->get_tx_antenna() << "\n";
    } else {
        std::cout << "[Config] Bistatic mode ACTIVE (TX: " << usrp->get_tx_antenna() 
                << ", RX: " << usrp->get_rx_antenna() << ")\n";
    }
    // --- Generate TX signal from config ---
    std::vector<std::complex<int16_t>> signal_tx = generate_signal_tx(cfg);


    const double pri          = 1.0 / cfg.prf;
    const double t_min_s      = cfg.t_min_us * 1e-6;
    const double t_max_s      = cfg.t_max_us * 1e-6;
    const double window_dur   = t_max_s - t_min_s;
    const size_t window_samps = static_cast<size_t>(window_dur * cfg.fs);
    const size_t total_pulses = static_cast<size_t>(cfg.session_duration * cfg.prf);
    const size_t total_samps  = total_pulses * window_samps;

    // --- Initialize Streamers (BEFORE setting hardware time) ---
    // Creating streamers here prevents hardware clock negotiation delays later
// --- Compute RX Window Size ---

    // --- Initialize Streamers (BEFORE setting hardware time) ---
    // RX streamer needs 'spb' matched to window_samps to align hardware USB buffers
    uhd::stream_args_t rx_stream_args("sc16", "sc16");
    rx_stream_args.channels = {0};
    rx_stream_args.args["spb"] = std::to_string(window_samps);

    uhd::stream_args_t tx_stream_args("sc16", "sc16");
    tx_stream_args.channels = {0};

    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

    // --- Shared time reference ---
    // Zero the USRP clock, then schedule both threads from the same t_start
    usrp->set_time_now(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Allow FPGA registers to settle

    uhd::time_spec_t t_start = usrp->get_time_now() + uhd::time_spec_t(cfg.start_offset_s);

    std::cout << "[Main] Session start in " << cfg.start_offset_s * 1e3 << " ms\n"
              << "[Main] Launching TX and RX threads...\n";



    // --- Launch threads ---
    std::atomic<bool> rx_ready(false);
    std::atomic<bool> stop_flag(false);



    std::cout << "[Main] Total pulse: " << total_pulses      << "\n"
              << "[Main] Total samps: " << total_samps       << "\n"
              << "[Main] Buffer size: " << (total_samps * sizeof(std::complex<int16_t>)) / 1e6
              << " MB\n";

    // --- 3. Allocate full session buffer ---
    std::vector<std::complex<int16_t>> buffer;
    try {
        buffer.resize(total_samps);
    }
    catch (const std::bad_alloc& e) {
        throw std::runtime_error("[Main] Not enough memory for session buffer: "
                                 + std::string(e.what()));
    }

    // Pass rx_stream into rx_thread
    std::thread rx_thread(rx_thread_func, usrp, rx_stream, cfg.prf,
                          cfg.session_duration, cfg.fs,
                          cfg.t_min_us, cfg.t_max_us,
                          std::ref(t_start), std::ref(stop_flag),
                          std::ref(rx_ready), std::ref(buffer));

    pin_thread(rx_thread, 3);

    // Wait for RX to finish initializing and queuing initial commands
    while (!rx_ready) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::cout << "[Main] RX ready — launching TX\n";

    // Pass tx_stream into tx_thread
    std::thread tx_thread(tx_thread_func, usrp, tx_stream, cfg.prf,
                          cfg.session_duration, cfg.fs,
                          t_start, std::ref(stop_flag), std::ref(signal_tx));

    pin_thread(tx_thread, 2);
    
    tx_thread.join();
    rx_thread.join();

    // --- Cleanup ---
    usrp.reset();
    std::cout << "[Main] Session complete. Disconnected.\n";
    return 0;
}
