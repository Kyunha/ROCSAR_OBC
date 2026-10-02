#include <chrono>
#include <ctime>
#include <thread>
#include <vector>
#include <complex>
#include <iostream>
#include <fstream>
#include <atomic>
#include <algorithm>
#include <stdexcept>
#include <uhd/utils/thread.hpp>

#include "libs/rx/rx.hpp"

void rx_thread_func(
    uhd::usrp::multi_usrp::sptr    usrp,
    uhd::rx_streamer::sptr         rx_stream,
    double                         prf,
    double                         session_dur,
    double                         fs,
    double                         t_min_us,
    double                         t_max_us,
    const uhd::time_spec_t&        t_start,
    std::atomic<bool>&             stop_flag,
    std::atomic<bool>&             rx_ready,
    std::vector<std::complex<int16_t>>& buffer
)
{
    // --- 2. Timing / window ---
    const double pri          = 1.0 / prf;
    const double t_min_s      = t_min_us * 1e-6;
    const double t_max_s      = t_max_us * 1e-6;
    const double window_dur   = t_max_s - t_min_s;
    const size_t window_samps = static_cast<size_t>(window_dur * fs);
    const size_t total_pulses = static_cast<size_t>(session_dur * prf);
    const size_t total_samps  = total_pulses * window_samps;

    // Set socket timeout to 1.0s to allow hardware t_start offset to elapse without timing out early
    const double recv_timeout = 1.0;

    std::cout << "[RX] Window      : " << t_min_us          << " - " << t_max_us << " us\n"
              << "[RX] Window dur : " << window_dur * 1e6  << " us\n"
              << "[RX] Win samples: " << window_samps      << "\n"
              << "[RX] Timeout val: " << recv_timeout      << " s\n";

    // --- 4. Pre-compute RX window timestamps ---
    std::vector<uhd::time_spec_t> rx_times(total_pulses);
    for (size_t i = 0; i < total_pulses; ++i) {
        rx_times[i] = t_start
                    + uhd::time_spec_t(i * pri)
                    + uhd::time_spec_t(t_min_s);
    }

    // --- 5. Pin thread priority ---
    uhd::set_thread_priority_safe(1.0f, true);

    // --- 6. Pre-fill the command queue ---
    const size_t CMD_AHEAD = 16; 
    size_t cmd_idx = 0;

    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    cmd.num_samps  = window_samps;
    cmd.stream_now = false;

    for (; cmd_idx < std::min(total_pulses, CMD_AHEAD); ++cmd_idx) {
        cmd.time_spec = rx_times[cmd_idx];
        rx_stream->issue_stream_cmd(cmd);
    }

    // Signal main thread that RX setup is complete
    rx_ready = true;

    // --- 7. Pulse loop ---
    size_t pulses_recvd        = 0;
    size_t total_timeouts      = 0;
    size_t total_overflows     = 0;
    size_t total_late_cmds     = 0;
    size_t total_broken_chains = 0;
    size_t total_align_errs    = 0;
    size_t total_other_errs    = 0;
    double worst_recv_ms       = 0.0;

    for (size_t pulse = 0; pulse < total_pulses && !stop_flag; ++pulse)
    {
        std::complex<int16_t>* pulse_ptr = buffer.data() + pulse * window_samps;
        uhd::rx_metadata_t md;
        size_t samps_recvd = 0;

        const auto t0 = std::chrono::steady_clock::now();

        while (samps_recvd < window_samps && !stop_flag)
        {
            size_t n = rx_stream->recv(
                pulse_ptr + samps_recvd,
                window_samps - samps_recvd,
                md,
                recv_timeout
            );

            // Accumulate errors silently to prevent console logging stalls in RT execution
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                switch (md.error_code) {
                    case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
                        ++total_timeouts;
                        break;
                    case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
                        ++total_overflows;
                        break;
                    case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
                        ++total_late_cmds;
                        break;
                    case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
                        ++total_broken_chains;
                        break;
                    case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT:
                        ++total_align_errs;
                        break;
                    default:
                        ++total_other_errs;
                        break;
                }

                if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                    break;
                }
            }

            samps_recvd += n;
        }

        const auto t1 = std::chrono::steady_clock::now();
        const double recv_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (recv_ms > worst_recv_ms) worst_recv_ms = recv_ms;

        // Issue the next stream command to maintain queue pipeline depth
        if (cmd_idx < total_pulses && !stop_flag) 
        {
            cmd.time_spec = rx_times[cmd_idx];
            rx_stream->issue_stream_cmd(cmd);
            cmd_idx++;
        }

        ++pulses_recvd;
    }

    // --- 8. Session done — write to file ---
    std::cout << "[RX] Capture done.\n"
              << "[RX] Pulses received : " << pulses_recvd        << " / " << total_pulses << "\n"
              << "[RX] Timeouts        : " << total_timeouts      << "\n"
              << "[RX] Overflows       : " << total_overflows     << "\n"
              << "[RX] Late Commands   : " << total_late_cmds     << "\n"
              << "[RX] Broken Chains   : " << total_broken_chains << "\n"
              << "[RX] Alignment Errs  : " << total_align_errs    << "\n"
              << "[RX] Other Errors    : " << total_other_errs    << "\n"
              << "[RX] Worst recv()    : " << worst_recv_ms       << " ms  (budget: " << pri * 1e3 << " ms)\n"
              << "[RX] Writing to disk...\n";

    auto now        = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm      = *std::localtime(&time_t_now);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);
    std::string filename = std::string("./../sdr-ettus-b200mini/Data/rx_data_") + timestamp + ".bin";

    size_t total_bytes = static_cast<size_t>(pulses_recvd) * window_samps * sizeof(std::complex<int16_t>);
    
    if (total_bytes > buffer.size() * sizeof(buffer[0])) {
        throw std::runtime_error("[RX] Error: Requested write size exceeds allocated buffer size!");
    }

    std::ofstream outfile(filename, std::ios::binary | std::ios::trunc);
    if (!outfile.is_open()) {
        throw std::runtime_error("[RX] Failed to open " + filename);
    }

    const char* data_ptr = reinterpret_cast<const char*>(buffer.data());
    constexpr size_t CHUNK_SIZE = 32 * 1024 * 1024; // 32 MB per chunk
    std::chrono::milliseconds sleep_duration(10); /** Sleep for 10 ms between each chunk write, to avoid overwhelming the CPU */


    for (size_t offset = 0; offset < total_bytes; offset += CHUNK_SIZE) {
        size_t bytes_to_write = std::min(CHUNK_SIZE, total_bytes - offset);
        
        outfile.write(data_ptr + offset, bytes_to_write);
        
        if (!outfile) {
            throw std::runtime_error("[RX] Disk write failed mid-stream!");
        }
        
        outfile.flush(); 

        std::this_thread::sleep_for(sleep_duration);
    }

    outfile.close();
    std::cout << "[RX] Saved to " << filename << "\n";
}