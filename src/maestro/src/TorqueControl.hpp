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
  void set_GainAgan(short* reg, int gainStartId_mbus, bool times10000 = true);
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

  double get_pos() const { return now_pos; }
  double get_vel() const { return now_vel; }

private:
  CMMCSingleAxis axis;

  double kp; // [/s]
  double kd; // [mA/(cnt/s)]
  double ki; // [/ms]
  int target_pos = 0;
  int torLim_mA = 6000;
  int target_pos_old;

  MMC_CONNECT_HNDL conn_handle;

  // For temporals to use torControl
  double now_pos, now_vel;
  double tor_order, tor_order_integral;
  bool torLimFlag;
  // User utils
  int read_32bit_from_mbus16bit(short* reg, int startRef);
};
