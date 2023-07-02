#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <string>

#include "../external/spdlog/include/spdlog/sinks/basic_file_sink.h"    // File logger
#include "../external/spdlog/include/spdlog/sinks/stdout_color_sinks.h" // Multi-colored console logger
#include "../external/spdlog/include/spdlog/spdlog.h"


#if 0
inline std::string get_logger_filename() {
  auto now       = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  return ss.str();
}

inline void init_logger() {
  const auto fname = get_logger_filename();

  std::filesystem::create_directory("logs");

  try {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>("console", true));
    sinks.push_back(std::make_shared<spdlog::sinks::simple_file_sink_mt>("logfile", "txt"));
    auto combined_logger = std::make_shared<spdlog::logger>("main", begin(sinks), end(sinks));

    auto stdout_sink = spdlog::sinks::stdout_sink_mt::instance();
    auto color_sink  = std::make_shared<spdlog::sinks::ansicolor_sink>(stdout_sink);
    sinks.push_back(color_sink);
    sinks.push_back(std::make_shared<spdlog::sinks::simple_file_sink_mt>("logfile.txt"));
    spdlog::register_logger(combined_logger);
    combined_logger->info("Welcome !");
  } catch(const spdlog::spdlog_ex& ex) {
    std::cout << "Log failed: " << ex.what() << std::endl;
  } catch(...) {
    std::cout << "Log failed: " << std::endl;
  }
}
#endif
