#include <h4rc/ModbusMaster.hpp>

namespace h4rc {

bool ModbusServer::start() {
  time_t timer;
  int rc;
  int ret = EXIT_SUCCESS;
  uint16_t tab_rp_registers;

  m_ctx = modbus_new_rtu(m_devicename.c_str(), m_serial_speed, m_parity, m_databit, m_stopbit);

  if(m_ctx == NULL) {
    fprintf(stderr, "Unable to allocate libmodbus context\n"
                    "modbus error\n");
    return false;
  }

  modbus_set_debug(m_ctx, FALSE);
  // modbus_set_error_recovery(m_ctx, MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL);

  if(modbus_set_slave(m_ctx, m_slave_id) == -1) {
    fprintf(stderr,
            "set slave failed: %s\n"
            "modbus error\n",
            modbus_strerror(errno));
    modbus_free(m_ctx);
    return false;
  }

  return true;
}

bool ModbusServer::connect() {
  if(modbus_connect(m_ctx) == -1) {
    fprintf(stderr,
            "Connection failed: %s\n"
            "modbus error\n",
            modbus_strerror(errno));
    modbus_free(m_ctx);
    return false;
  }
  return true;
}

bool ModbusServer::read(uint16_t* tab_rp_registers, int nb) {
  int rc;
  rc = modbus_read_registers(m_ctx, m_register_address, nb, tab_rp_registers);
  if(rc == -1) {
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return false;
  }
  return true;
}

bool ModbusServer::write(uint16_t* tab_rp_registers, int nb) {
  int rc;
  rc = modbus_write_registers(m_ctx, m_register_address, nb, tab_rp_registers);
  if(rc == -1) {
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return false;
  }
  return true;
}

bool ModbusServer::disconnect() {
  modbus_close(m_ctx);
  modbus_free(m_ctx);
  return true;
}

ModbusServer::ModbusServer() {}
ModbusServer::~ModbusServer() {}

} // namespace h4rc
