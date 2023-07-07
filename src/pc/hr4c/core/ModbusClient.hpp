#pragma once
#include <array>
#include <cassert>
#include <iostream>
#include <modbus/modbus.h>

// https://armadillo.atmark-techno.com/howto/armadillo-400-modbus

#define MODBUS_AXIS_DATA_NUM 60
#define MODBUS_COMMAND_DATA_NUM 50

namespace hr4c {

class ModbusClient {
protected:
  ModbusClient();
  ~ModbusClient();
  static ModbusClient* singleton_;

private:
  modbus_t* m_ctx;
  const std::string m_devicename = "/dev/ttyUSB0";
  const int m_slave_id           = 1;
  const int m_register_address   = 0;

  std::array<uint16_t, MODBUS_AXIS_DATA_NUM> m_tab_reg;
  std::array<uint16_t, MODBUS_COMMAND_DATA_NUM> m_cmd_reg;
  bool m_connected = false;

public:
  // no copyable
  ModbusClient(ModbusClient& other)            = delete;
  ModbusClient(ModbusClient&& other)           = delete;
  ModbusClient& operator=(const ModbusClient&) = delete;
  ModbusClient& operator=(ModbusClient&&)      = delete;

  bool start(const std::string ip, int port);
  bool connect();

  bool send_axis_data() const;
  bool send_command_data() const;
  bool read_axis_data();
  bool read_command_data();

  template <typename T> T read_axis_data(int index) const {
    assert(index < MODBUS_AXIS_DATA_NUM);
    T value;
    int16_t* ptr = static_cast<int16_t*>((void*)&value);
    for(size_t i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      *(ptr + i) = m_tab_reg[index + i];
    }
    return value;
  }
  template <typename T> T read_command_data(int index) const {
    assert(index < MODBUS_COMMAND_DATA_NUM);
    T value;
    int16_t* ptr = static_cast<int16_t*>((void*)&value);
    for(size_t i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      *(ptr + i) = m_cmd_reg[index + i];
    }
    return value;
  }

  template <typename T> void set_axis_data(int index, const T data) {
    assert(index < MODBUS_AXIS_DATA_NUM);
    const void* data_ptr = static_cast<const void*>(&data);
    for(size_t i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      m_tab_reg[index + i] = *(static_cast<const uint16_t*>(data_ptr) + i);
    }
  }
  template <typename T> void set_command_data(int index, const T data) {
    assert(index < MODBUS_AXIS_DATA_NUM);
    const void* data_ptr = static_cast<const void*>(&data);
    for(size_t i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      m_cmd_reg[index + i] = *(static_cast<const uint16_t*>(data_ptr) + i);
    }
  }

  bool disconnect();

  static ModbusClient* Get() {
    if(!singleton_) singleton_ = new ModbusClient();
    return singleton_;
  }
};


} // namespace hr4c
