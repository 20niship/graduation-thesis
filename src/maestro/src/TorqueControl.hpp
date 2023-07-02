#pragma once

#include "mmcpplib.h"
#include <MMC_definitions.h>

class TorControls {
public:
  TorControls() = default;
  TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard = 5500);
  ~TorControls() = default;
  void set_CommandFromHost(short* reg, int torlimMbusId);
  void p_pi_controlAxis();
  void reset_integral();
  int get_currentLim() const { return std::max(torLim_mA, 0); }
  double get_KP_pos() const { return kp; }
  double get_KP_vel() const { return kd; }
  double get_tor_order() const { return tor_order; }
  void set_target(double p) { target_pos = p; }
  void set_limit(double l) { torLim_mA = l; }

  bool init(const std::string& axisName, const MMC_CONNECT_HNDL& gConnHndl);
  bool poweron();
  void abort();
  bool poweroff();
  bool check_status();

  //! @brief update the state of the axis
  bool update(){
    sync_state();
    return true;
  }

  double get_pos() const { return now_pos; }
  double get_vel() const { return now_vel; }

private:
  void sync_state();

  CMMCSingleAxis axis;
  std::string m_axisName;

  double kp; // [/s]
  double kd; // [mA/(cnt/s)]
  double ki; // [/ms]
  int target_pos = 0;
  int torLim_mA  = 500;
  int target_pos_old;

  MMC_CONNECT_HNDL conn_handle;

  double now_pos, now_vel;
  double tor_order, tor_order_integral;
  bool torLimFlag;
};
