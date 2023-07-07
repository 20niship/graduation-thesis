#include <cmath>

#include <hr4c/core/AxisInterface.hpp>
#include <hr4c/core/ModbusClient.hpp>
#include <modbus/modbus.h>

#include "../../../common/communication_const.hpp"

using namespace std;

namespace hr4c {

AxisInterface::AxisInterface(int mbusRefId) {
  m_modbus_id = mbusRefId;
  auto server = ModbusClient::Get();
  if(!server) {
    cout << "ModbusServer is not initialized" << endl;
    exit(1);
  }
  server->read_axis_data();
  m_start_pos = server->read_axis_data<int32_t>(eAx1 + 0);
  cout << "init_pos_of_elmo inpt axis :" << m_start_pos << endl;
  enc0_slave_rad_is_loaded = false;
}

AxisInterface::~AxisInterface() {}

void AxisInterface::set_target_pos(double target) {
  auto server = ModbusClient::Get();
  if(!server) {
    cout << "ModbusServer is not initialized" << endl;
    exit(1);
  }
  server->set_command_data<int32_t>(m_modbus_id * 8 + 6, target);
  server->send_axis_data();
}

void AxisInterface::update_sensor() {
  auto server = ModbusClient::Get();
  if(!server) {
    cout << "ModbusServer is not initialized" << endl;
    exit(1);
  }
  server->read_axis_data();
  auto t_pos = server->read_axis_data<int32_t>(m_modbus_id + eActualPos);
  auto t_vel = server->read_axis_data<int32_t>(m_modbus_id + eActualVel);
  auto t_cur = server->read_axis_data<int32_t>(m_modbus_id + eActualTor);
  if(t_pos != 0 && t_vel != 0 && t_cur != 0) {
    pos = t_pos;
    vel = t_vel;
    cur = t_cur;
  }
}

double AxisInterface::get_motorOtptAxis_rad() { return (pos - m_start_pos) * (2.0 * M_PI / 4095.) + enc0_slave_rad; }

bool AxisInterface::poweron() { return true; }
bool AxisInterface::poweroff() { return true; }

} // namespace hr4c
