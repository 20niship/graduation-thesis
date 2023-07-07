#include <hr4c/core/AxisInterface.hpp>
#include <hr4c/core/ModbusClient.hpp>
#include <hr4c/core/h4rc.hpp>
#include <hr4c/core/logger.hpp>
#include <hr4c/core/robot.hpp>

namespace hr4c {

bool start(const std::string& ip, int port) {
  init_logger();
  auto server = ModbusClient::Get();
  if(!server->start(ip, port)) return false;
  return server->connect();
}

bool terminate() {
  auto server = ModbusClient::Get();
  // TODO: send disconnect command!
  server->disconnect();
  return true;
}

} // namespace hr4c
