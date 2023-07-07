#pragma once

#include "mmcpplib.h"
#include <MMC_definitions.h>

class TorControls {
public:
  TorControls() = default;
  TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard = 5500);
  ~TorControls() = default;
  void set_CommandFromHost(short* reg, int torlimMbusId);
  void p_pi_controlAxis(bool verbose = false);
  void reset_integral();
  int get_currentLim() const { return std::max(torLim_mA, 0); }
  double get_KP() const { return kp; }
  double get_KD() const { return kd; }
  double get_KI() const { return ki; }
  void set_KP(double p) { kp = p; }
  void set_KD(double d) { kd = d; }
  void set_KI(double i) { ki = i; }
  double get_tor_order() const { return tor_order; }
  void set_target(double p) { target_pos = p; }
  void set_limit(double l) { torLim_mA = l; }

  bool init(const std::string& axisName, const MMC_CONNECT_HNDL& gConnHndl);
  bool poweron();
  void abort();
  bool poweroff();
  bool check_status();

  //! @brief update the state of the axis
  bool update() {
    sync_state();
    return true;
  }

  double get_pos() const { return now_pos; }
  double get_vel() const { return now_vel; }

  void goto_home();

private:
  void sync_state();

  CMMCSingleAxis axis;
  std::string m_axisName;

  double kp         = 0; // [/s]
  double kd         = 0; // [mA/(cnt/s)]
  double ki         = 0; // [/ms]
  double target_pos = 0;
  int torLim_mA     = 200;
  int target_pos_old;

  MMC_CONNECT_HNDL conn_handle;

  double now_pos, now_vel;
  double tor_order, tor_order_integral;
  bool torLimFlag;

  double pos_error      = 0;
  double last_pos_error = 0;
  bool is_power_on      = false;
};
