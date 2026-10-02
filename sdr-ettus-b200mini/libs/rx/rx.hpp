#ifndef RX_HPP
#define RX_HPP
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/time_spec.hpp>
#include <atomic>
#include <vector>
#include <complex>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <complex>

void rx_thread_func(
    uhd::usrp::multi_usrp::sptr  usrp,
    uhd::rx_streamer::sptr         rx_stream,
    double                        prf,
    double                        session_dur,
    double                        fs,
    double                        t_min_us,
    double                        t_max_us,
    const uhd::time_spec_t&       t_start,
    std::atomic<bool>&            stop_flag,
    std::atomic<bool>&             rx_ready,
    std::vector<std::complex<int16_t>>& buffer
);

#endif // RX_HPP