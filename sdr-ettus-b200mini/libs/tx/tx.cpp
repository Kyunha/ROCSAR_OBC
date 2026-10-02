#include "libs/tx/tx.hpp"
#include <uhd/utils/thread.hpp>
#include <chrono>

void tx_thread_func(
    uhd::usrp::multi_usrp::sptr                        usrp,
    uhd::tx_streamer::sptr                             tx_stream,
    double                                             prf,
    double                                             session_dur,
    double                                             fs,
    uhd::time_spec_t                                   t_start,
    std::atomic<bool>&                                 stop_flag,
    const std::vector<std::complex<int16_t>>&          signal_tx
)
{


    // --- 2. Timing parameters ---
    const double   pri          = 1.0 / prf;
    const size_t   pulse_samps  = signal_tx.size();
    const size_t   total_pulses = static_cast<size_t>(session_dur * prf);
    const size_t   pri_samps    = static_cast<size_t>(pri * fs);

    std::cout << "[TX] PRF         : " << prf                    << " Hz\n"
              << "[TX] PRI         : " << pri * 1e3              << " ms\n"
              << "[TX] PRI samples : " << pri_samps              << "\n"
              << "[TX] Pulse len   : " << pulse_samps            << " samples\n"
              << "[TX] Total pulses: " << total_pulses           << "\n"
              << "[TX] Duty cycle  : " << 100.0 * pulse_samps / pri_samps << " %\n";

    // --- 3. Pre-compute all timestamps ---
    std::vector<uhd::time_spec_t> pulse_times(total_pulses);
    for (size_t i = 0; i < total_pulses; ++i)
        pulse_times[i] = t_start + uhd::time_spec_t(i * pri);

    // --- 4. Pin thread priority ---
    uhd::set_thread_priority_safe();

    // --- 5. TX metadata ---
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec      = pulse_times[0];

    size_t pulses_sent    = 0;
    size_t pulses_dropped = 0;
    double worst_send_ms  = 0.0;

    // --- 6. Pulse loop ---
    for (size_t pulse = 0; pulse < total_pulses && !stop_flag; ++pulse)
    {
        // md.time_spec      = pulse_times[pulse];
        md.start_of_burst = (pulse == 0);
        md.end_of_burst   = (pulse == total_pulses - 1);

        // Measure send() blocking time
        const auto t0 = std::chrono::steady_clock::now();

        const size_t sent = tx_stream->send(
            signal_tx.data(),
            signal_tx.size(),  // full PRI
            md//,
            //2.0
        );

        const auto t1 = std::chrono::steady_clock::now();
        const double send_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (send_ms > worst_send_ms) worst_send_ms = send_ms;

        // Check async error messages (non-blocking)
        uhd::async_metadata_t async_md;
        if (tx_stream->recv_async_msg(async_md, 0.0)) {
            if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_TIME_ERROR)
                std::cerr << "[TX] Late timestamp on pulse " << pulse << "\n";
            if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW)
                std::cerr << "[TX] Underflow on pulse " << pulse << "\n";
        }

        if (sent != pulse_samps) {
            std::cerr << "[TX] Warning: sent " << sent
                      << "/" << pulse_samps << " on pulse " << pulse << "\n";
            ++pulses_dropped;
        } else {
            ++pulses_sent;
        }

        md.start_of_burst = false;
        md.has_time_spec  = false;  // only the first pulse has a timestamp
    }

    // --- 7. Send EOB ---
    uhd::tx_metadata_t eob;
    eob.end_of_burst = true;
    tx_stream->send("", 0, eob);

    std::cout << "[TX] Done.\n"
              << "[TX] Pulses sent     : " << pulses_sent    << " / " << total_pulses << "\n"
              << "[TX] Pulses dropped  : " << pulses_dropped << "\n"
              << "[TX] Success rate    : " << (static_cast<double>(pulses_sent) / total_pulses) * 100.0 << " %\n"
              << "[TX] Worst send()    : " << worst_send_ms  << " ms  (budget: " << pri * 1e3 << " ms)\n"
              << "[TX] Session end time: " << usrp->get_time_now().get_real_secs() << " s\n";
}
