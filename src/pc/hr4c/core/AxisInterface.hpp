#pragma once
#include <iostream>

namespace hr4c {

class AxisInterface {
public:
  AxisInterface(int mbusId);
  ~AxisInterface();
  bool is_connected;

  void send_cmd(float rad_edata, int Torlim);
  void update_sensor();

  int get_pos() const { return pos; }
  int get_vel() const { return vel; }
  int get_cur() const { return cur; }

  bool poweron();
  bool poweroff();

  void set_gain(double kp, double kd, double ki);
  void set_target_pos(double pos);
  double get_motorOtptAxis_rad();

private:
  bool m_dir_positive = true;
  int direction_mode_plus_or_minus;
  int mbus_timestamp[4];
  bool enc0_slave_rad_is_loaded = false;
  // Read from modbus
  int32_t pos;
  int32_t vel;
  int32_t cur;
  int m_modbus_id;
  int m_start_pos;
  double enc0_slave_rad;
  int Torlim_straged_id_mbus;

  // user utils
  void set_ElmoCmd(int TorCmd, int PosCmd, uint16_t* reg);
};

} // namespace hr4c
