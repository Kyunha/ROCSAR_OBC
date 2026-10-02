#ifndef TX_HPP
#define TX_HPP
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/time_spec.hpp>
#include <atomic>
#include <vector>
#include <complex>
#include <iostream>
#include <thread>
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
);

#endif // TX_HPP