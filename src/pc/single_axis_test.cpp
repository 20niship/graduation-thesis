#include <limits>

#include <cpptoml.h>

#include <iostream>
#include <spdlog/spdlog.h>

#include <hr4c/ModbusClient.hpp>
#include <hr4c/common.hpp>
#include <hr4c/h4rc.hpp>

std::tuple<std::string, int> get_ip_port(const std::string& tomlfile) {
  auto config = cpptoml::parse_file(tomlfile);
  auto socket = config->get_table("socket");
  auto ip     = socket->get_as<std::string>("ip");
  auto port   = socket->get_as<int>("port");
  if(!ip || !port) {
    std::cout << "Error: ip or port not found in " << tomlfile << std::endl;
    std::exit(1);
  }
  return std::make_tuple(*ip, *port);
}

int main(int argc, char* argv[]) {
  auto fname      = (argc == 2) ? argv[1] : "../config.toml";
  auto [ip, port] = get_ip_port(fname);

  auto res = hr4c::start(ip, port);
  if(!res) {
    spdlog::error("Could not start server");
    hr4c::terminate();
    return 1;
  }

  std::vector<hr4c::AxisInterface> axes;
  axes.emplace_back(hr4c::eAx1);
  axes.emplace_back(hr4c::eAx2);

  // axis.poweron();

  while(!kbhit()) {
    for(auto& axis : axes) {
      axis.update_sensor();
      auto pos = axis.get_pos();
      auto v   = axis.get_vel();
      auto c   = axis.get_cur();
      spdlog::info("pos: {}, vel: {}, cur: {}", pos, v, c);
    }
    static int i = 0;
    static int k = 0;
    if(i++ % 100 == 0) {
      auto client = hr4c::ModbusClient::Get();
      client->set_command_data<int16_t>(hr4c::eCommand1, k);
      client->set_command_data<int32_t>(hr4c::eCommand2, k * 10000);
      client->set_command_data<int32_t>(hr4c::eCommand3, k * 1e9);
      client->send_command_data();
      k++;
      i = 0;
    }
  }

  hr4c::terminate();
  return 0;
}
