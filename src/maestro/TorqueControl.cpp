#include <algorithm>
#include <math.h>

#include "TorqueControl.hpp"
#include "logger.h"

TorControls::TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard) {
  // TODO Auto-generated constructor stub
  kp = kp_pos / (2.0 * M_PI); // [rad/s] to [/s]
  kd = kp_vel * 1000.;        // [A/(cnt/s)] to [mA/(cnt/s)]
  ki = ki_vel / 1000.;        // [Hz] = [1/s] to [1/ms]
  // initialize private variables
  tor_order_integral = 0;
  torLimFlag         = false;
}


void TorControls::set_GainAgan(short* reg, int gainStartId_mbus, bool times10000) {
  // Define id
  int kp_pos_MbusId = gainStartId_mbus;
  int kp_vel_MbusId = gainStartId_mbus + 2;
  int ki_vel_MbusId = gainStartId_mbus + 4;

  // Read from mbus
  double tmp_kp_pos = read_32bit_from_mbus16bit(reg, kp_pos_MbusId) * 1.0;
  double tmp_kp_vel = read_32bit_from_mbus16bit(reg, kp_vel_MbusId) * 1.0;
  double tmp_ki_vel = read_32bit_from_mbus16bit(reg, ki_vel_MbusId) * 1.0;

  // if multiplied 10000
  if(times10000) {
    tmp_kp_pos = 1.0 * tmp_kp_pos / 100.;
    tmp_kp_vel = 1.0 * tmp_kp_vel / 10000000.;
    tmp_ki_vel = 1.0 * tmp_ki_vel / 100.;
  }
  kp = 1.0 * tmp_kp_pos / (2.0 * M_PI); // [rad/s] to [/s]
  kd = 1.0 * tmp_kp_vel * 1000.;        // [A/(cnt/s)] to [mA/(cnt/s)]
  ki = 1.0 * tmp_kp_vel / 1000.;        // [Hz] = [1/s] to [1/ms]
}

void TorControls::set_CommandFromHost(short* reg, int torlimMbusId) {
  int posCmdMbusId = torlimMbusId + 2;
  torLim_mA        = read_32bit_from_mbus16bit(reg, torlimMbusId);
  target_pos       = read_32bit_from_mbus16bit(reg, posCmdMbusId);
  if(target_pos == 0) {
    // It means modbus connection is failed tamporary
    target_pos = target_pos_old;
  }
  return;
}


int TorControls::read_32bit_from_mbus16bit(short* reg, int startRef) {
  int hi  = (reg[startRef] << 16) >> 16;
  int lo  = (reg[startRef + 1] << 16) >> 16;
  int ret = hi * 32767 + lo;
  return ret;
}

void TorControls::reset_integral() {
  cout << "RES";
  tor_order_integral = 0;
  tor_order          = 0;
  torLimFlag         = false;
}

void TorControls::p_pi_controlAxis() {
  now_pos              = axis.GetActualPosition();
  now_vel              = axis.GetActualVelocity();
  const double pos_err = (target_pos * 1.0 - now_pos);
  const double lim_mA  = get_currentLim();

  std::cout << "[pos] " << now_pos << " [vel] " << now_vel << " [tar] " << target_pos << " [tor] " << tor_order << " [lim] " << lim_mA << " [int] " << tor_order_integral << std::endl;

  // 1. set velocity
  const auto v_order = kp * pos_err;
  tor_order          = kd * (v_order - now_vel + tor_order_integral);

  if(torLimFlag == false) {
    tor_order_integral += ki * (v_order - now_vel);
  }

  tor_order = std::min(std::max(tor_order, -lim_mA), lim_mA);

  axis.MoveTorque(tor_order, 5.0 * pow(10, 6), 1.0 * pow(10, 8), MC_ABORTING_MODE);
  //			single_axis.MoveAbsoluteRepetitive(target_pos,MC_ABORTING_MODE);
  //			cout <<"vOrd" << v_order <<  "tOrd" << tor_order << " ** ";
  // For Abnormal termination
  int axisStatus = axis.ReadStatus();
  if(axisStatus & NC_AXIS_ERROR_STOP_MASK) {
    axis.Reset();
    axisStatus = axis.ReadStatus();
    if(axisStatus & NC_AXIS_ERROR_STOP_MASK) {
      cout << "Axis a1 in Error Stop. Aborting.";
      exit(0);
    }
    target_pos_old = target_pos;
  }
  return;
}

void TorControls::init(const std::string& axisName, const MMC_CONNECT_HNDL& gConnHndl) {
  MMC_MOTIONPARAMS_SINGLE stSingleDefault; // Single axis default data
  // Initialize default parameters. This is not a must. Each parameter may be initialized individually.
  stSingleDefault.fEndVelocity  = 0;
  stSingleDefault.dbDistance    = 100000;
  stSingleDefault.dbPosition    = 0;
  stSingleDefault.fVelocity     = 100000;
  stSingleDefault.fAcceleration = 1000000;
  stSingleDefault.fDeceleration = 1000000;
  stSingleDefault.fJerk         = 20000000;
  stSingleDefault.eDirection    = MC_POSITIVE_DIRECTION;
  stSingleDefault.eBufferMode   = MC_BUFFERED_MODE;
  stSingleDefault.ucExecute     = 1;

  std::cout <<" initializing axis : " << axisName << std::endl;
  axis.InitAxisData(axisName.c_str(), gConnHndl);
  LOGI << 7 << LEND;
  axis.SetDefaultParams(stSingleDefault);
  LOGI << 8 << LEND;
  axis.m_fAcceleration = 10000;
  this->check_status();
}


void TorControls::poweron() {
  // mode change
  int ret = axis.SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
  std::cout << "change is successed if ret == 0, and ret is ..." << ret << endl;
  if(ret != 0) {
    std::cerr << "Set error" << endl;
  } else {
    std::cout << "Mode is 10 ? ... " << axis.GetOpMode() << endl;
  }
  // Axis idling state
  axis.PowerOn();
  this->check_status();
  cout << "power on is completed" << endl;
  return;
}

void TorControls::poweroff() {
  axis.PowerOff();
  int Status = axis.ReadStatus();
  if(Status & NC_AXIS_ERROR_STOP_MASK) {
    axis.Reset();
    Status = axis.ReadStatus();
    if(Status & NC_AXIS_ERROR_STOP_MASK) {
      LOGE << "Axis is in Error Stop. Aborting." << LEND;
      exit(0);
    }
    std::cout << "power OFF" << std::endl;
  }
  return;
}

int TorControls::check_status() {
  axis.PowerOff();
  int Status = axis.ReadStatus();
  if(Status & NC_AXIS_ERROR_STOP_MASK) {
    axis.Reset();
    LOGE <<" AXIS RESET CALLED!! sleeping....." << LEND;
    sleep(1);
    Status = axis.ReadStatus();
    if(Status & NC_AXIS_ERROR_STOP_MASK) {
      LOGE << "Axis is in Error Stop. Aborting." << LEND;
      exit(0);
    }
    std::cout << "power OFF" << std::endl;
  }
  return Status;
}
