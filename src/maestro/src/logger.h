#pragma once

#include <chrono>
#include <ctime>
#if 0
#include <filesystem>
#else
// for create direcroy  c
// TODO: disable this
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#if 0
#include "../external/spdlog/include/spdlog/sinks/basic_file_sink.h"
#include "../external/spdlog/include/spdlog/sinks/stdout_color_sinks.h"
#include "../external/spdlog/include/spdlog/spdlog.h"


inline std::string get_logger_filename() {
  auto now       = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
  return ss.str();
}

inline void init_logger() {
  const auto fname = get_logger_filename();
#if 0
  std::filesystem::create_directory("logs");
#else
  struct stat st = {0};
  if(stat("logs", &st) == -1) {
    mkdir("logs", 0700);
  }
  const auto path = "./logs/" + fname + ".log";
#endif

  try {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
    spdlog::logger logger("multi_sink", {console_sink, file_sink});
    logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    logger.set_level(spdlog::level::debug);
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(logger));
  } catch(const spdlog::spdlog_ex& ex) {
    std::cout << "Log failed: " << ex.what() << std::endl;
  } catch(...) {
    std::cout << "Log failed: " << std::endl;
  }
}
#else

namespace spdlog{
static constexpr auto prefix_i = "";
static constexpr auto prefix_d = "\x1b[32m";
static constexpr auto prefix_w = "\x1b[33m";
static constexpr auto prefix_e = "\x1b[31m";

static constexpr auto suffix_e = "\x1b[0m";

static constexpr auto type_i = "[ INF ] ";
static constexpr auto type_d = "[ DBG ] ";
static constexpr auto type_w = "[ WAR ] ";
static constexpr auto type_e = "[ ERR ] ";

#define LOGI std::cout << prefix_i << __FILE__ << " @ " << __LINE__ << type_i
#define LOGD std::cout << prefix_d << __FILE__ << " @ " << __LINE__ << type_d
#define LOGW std::cout << prefix_w << __FILE__ << " @ " << __LINE__ << type_w
#define LOGE std::cout << prefix_e << __FILE__ << " @ " << __LINE__ << type_e
#define LEND suffix_e << std::endl

inline void error(const std::string &msg,  ...){
  LOGE << msg << LEND;
}
inline void error(const std::string&msg, const std::string&hoge){
  error(msg + ", " + hoge);
}
inline void warn(const std::string &msg,  ...){
  LOGW << msg << LEND;
}
inline void info(const std::string &msg,  ...){
  std::cout << msg << std::endl;
}
inline void info(const std::string&msg, const std::string&hoge){
  info(msg + ", " + hoge);
}

inline void debug(const std::string &msg,  ...){
  LOGD << msg << LEND;
}
}
#endif

