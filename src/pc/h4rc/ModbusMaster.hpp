#pragma once
#include <iostream>
#include <modbus/modbus.h>

// https://armadillo.atmark-techno.com/howto/armadillo-400-modbus

namespace h4rc {
class ModbusServer {
protected:
  ModbusServer();
  ~ModbusServer();
  static ModbusServer* singleton_;

private:
    modbus_t *m_ctx;
    const std::string m_devicename = "/dev/ttyUSB0";
    const char m_parity = 'N';
    const int m_databit = 8;
    const int m_stopbit = 1;
    const int m_slave_id = 17;
    const int m_register_address = 8;
    const int m_serial_speed = 115200;
    
public:
  // no copyable
  ModbusServer(ModbusServer& other)            = delete;
  ModbusServer(ModbusServer&& other)           = delete;
  ModbusServer& operator=(const ModbusServer&) = delete;
  ModbusServer& operator=(ModbusServer&&)      = delete;

  bool start();
  bool connect();
  bool read(uint16_t* tab_rp_registers, int nb);
  bool write(uint16_t* tab_rp_registers, int nb);
  bool disconnect();

  static ModbusServer* Get() {
    if(!singleton_) singleton_ = new ModbusServer();
    return singleton_;
  }
};

} // namespace h4rc
