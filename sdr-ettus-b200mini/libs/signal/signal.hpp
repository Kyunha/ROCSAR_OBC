// signal.hpp
#ifndef SIGNAL_HPP
#define SIGNAL_HPP
#include <complex>
#include <vector>

struct Config;  // forward declaration

// Generate chirp signal based on configuration
std::vector<std::complex<int16_t>> generate_signal_tx(const Config& cfg);

#endif // SIGNAL_HPP