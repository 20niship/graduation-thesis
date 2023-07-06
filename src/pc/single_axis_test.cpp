#include <limits>
#include <cpptoml.h>

#include <common.hpp>
#include <hr4c/h4rc.hpp>
#include <iostream>
#include <spdlog/spdlog.h>

auto get_ip_port(const std::string& tomlfile) {
  auto config = cpptoml::parse_file(tomlfile);
  auto socket = config->get_table("socket");
  auto ip     = socket->get_as<std::string>("ip");
  auto port   = socket->get_as<int>("port");
  if(!ip || !port) {
    std::cout << "Error: ip or port not found in " << tomlfile << std::endl;
    std::exit(1);
  }
  return std::make_pair(*ip, *port);
}

int main(int argc, char** argv) {
  if(argc != 2) {
    std::cout << "Usage: " << argv[0] << " <filename>" << std::endl;
    return 1;
  }
  auto fname      = std::string(argv[1]);
  auto [ip, port] = get_ip_port(fname);

  auto res = hr4c::start(ip, port);
  if(!res) {
    spdlog::error("Could not start server");
    hr4c::terminate();
    return 1;
  }

  hr4c::AxisInterface axis(0);

  // axis.poweron();

  while(!kbhit()) {
    axis.update_sensor();
    auto pos = axis.get_pos();
    std::cout << "pos: " << pos << std::endl;
  }

  hr4c::terminate();
  return 0;
}
