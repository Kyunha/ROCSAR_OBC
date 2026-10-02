#pragma once
#include <string>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

struct Config {
    double prf;
    double fs;
    double session_duration;
    double tx_freq;
    double normalized_gain_tx;
    double normalized_gain_rx;
    double t_min_us;
    double t_max_us;
    double start_offset_s;
    double pulse_duration;
    double bw;
    std::string tx_antenna;
    std::string rx_antenna;
};



inline Config load_config(const std::string& path)
{
    std::ifstream f(path);
    std::cout << "[Config] Loading config from: " << path << "\n";
    if (!f.is_open())
        throw std::runtime_error("Cannot open config file: " + path);

    nlohmann::json j = nlohmann::json::parse(f);

    Config c;
    c.prf              = j.at("PRF").get<double>();
    c.fs               = j.at("FS").get<double>();
    c.session_duration = j.at("SESSION_DURATION").get<double>();
    c.tx_freq          = j.at("TX_FREQ").get<double>();
    c.normalized_gain_tx = j.at("NORMALIZED_GAIN_TX").get<double>();
    c.normalized_gain_rx = j.at("NORMALIZED_GAIN_RX").get<double>();
    c.t_min_us = j.at("T_MIN_US").get<double>();
    c.t_max_us = j.at("T_MAX_US").get<double>();
    c.start_offset_s = j.at("START_OFFSET_S").get<double>();
    // c.pulse_duration = j.at("PULSE_DURATION").get<double>();
    c.bw = j.at("BW").get<double>();
    c.tx_antenna = j.at("TX_ANTENNA").get<std::string>();
    c.rx_antenna = j.at("RX_ANTENNA").get<std::string>();

    if (c.start_offset_s <= 0.0)
        throw std::runtime_error("START_OFFSET_S must be positive");


    if (c.t_min_us >= c.t_max_us)
        throw std::runtime_error("T_MIN_US must be less than T_MAX_US");



    if (c.normalized_gain_tx < 0.0 || c.normalized_gain_tx > 1.0)
        throw std::runtime_error("NORMALIZED_GAIN_TX must be between 0.0 and 1.0");

    if (c.normalized_gain_rx < 0.0 || c.normalized_gain_rx > 1.0)
        throw std::runtime_error("NORMALIZED_GAIN_RX must be between 0.0 and 1.0");

    std::cout << "[Config] PRF              : " << c.prf              << " Hz\n"
              << "[Config] FS               : " << c.fs / 1e6        << " MSPS\n"
              << "[Config] Session duration : " << c.session_duration << " s\n"
              << "[Config] TX Freq          : " << c.tx_freq / 1e6   << " MHz\n"
              << "[Config] Normalized gain Tx  : " << c.normalized_gain_tx << "\n"
              << "[Config] Normalized gain Rx  : " << c.normalized_gain_rx << "\n"
              << "[Config] T_min            : " << c.t_min_us << " us\n"
              << "[Config] T_max            : " << c.t_max_us << " us\n"
              << "[Config] Start offset     : " << c.start_offset_s * 1e3 << " ms\n"
            //   << "[Config] Pulse duration   : " << c.pulse_duration * 1e6 << " us\n"
              << "[Config] Bandwidth        : " << c.bw / 1e6 << " MHz\n"
              << "[Config] TX Antenna       : " << c.tx_antenna << "\n"
              << "[Config] RX Antenna       : " << c.rx_antenna << "\n";

    return c;
}

// Uses nlohmann/json — single header, install on DietPi with:
// sudo apt install nlohmann-json3-dev
// And in your CMakeLists.txt:
// cmakefind_package(nlohmann_json REQUIRED)
// target_link_libraries(your_target nlohmann_json::nlohmann_json)