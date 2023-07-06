#include <cmath>

#include <hr4c/AxisInterface.hpp>
#include <hr4c/ModbusMaster.hpp>
#include <modbus/modbus.h>

using namespace std;

namespace hr4c {

AxisInterface::AxisInterface(int mbusRefId) {
  m_modbus_id = mbusRefId;
  auto server = ModbusClient::Get();
  if(!server) {
    cout << "ModbusServer is not initialized" << endl;
    exit(1);
  }

  m_start_pos = server->read_axis_data<int>(m_modbus_id * 8 + 0);
  cout << "init_pos_of_elmo inpt axis :" << m_start_pos << endl;
  enc0_slave_rad_is_loaded = false;
}

AxisInterface::~AxisInterface() {}

void AxisInterface::set_ElmoCmd(int TorCmd, int PosCmd, uint16_t* reg) {
  /* send_32bit_to_mbus16bit(TorCmd, reg, 0); */
  /* send_32bit_to_mbus16bit(PosCmd, reg, 2); */
}

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
  pos = server->read_axis_data<int32_t>(m_modbus_id * 8 + 0);
  vel = server->read_axis_data<int32_t>(m_modbus_id * 8 + 2);
  cur = server->read_axis_data<int32_t>(m_modbus_id * 8 + 4);
}

double AxisInterface::get_motorOtptAxis_rad() { return (pos - m_start_pos) * (2.0 * M_PI / 4095.) + enc0_slave_rad; }

bool AxisInterface::poweron() { return true; }
bool AxisInterface::poweroff() { return true; }

} // namespace hr4c
