#include <algorithm>
#include <math.h>

#include "TorqueControl.hpp"
#include "logger.h"

#ifdef WIN32
#define WAIT_SLEEP_MILLI(WAIT_MILLI_SEC) Sleep(WAIT_MILLI_SEC);
#else
#define WAIT_SLEEP_MILLI(WAIT_MILLI_SEC) usleep(WAIT_MILLI_SEC * 1000);
#endif

static int WaitFbDone(MMC_CONNECT_HNDL ComHndl, unsigned int break_state, CMMCSingleAxis* sng_axis) {
  int end_of = 0;
  int iCount = 0;
  unsigned int ulState;
  while(!end_of) {
    iCount++;
    end_of = 1;
    /* Read Axis Status command server for specific Axis */
    ulState = sng_axis->ReadStatus();
    if(!(ulState & break_state)) {
      end_of = 0;
      WAIT_SLEEP_MILLI(20);
    }
  }
  if(0) {
    MMC_SHOWNODESTAT_IN showin;
    MMC_SHOWNODESTAT_OUT showout;
    MMC_ShowNodeStatCmd(ComHndl, sng_axis->GetRef(), &showin, &showout);
  }
  return 0;
}

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
#if 0
MC_BUFFERED_MODE_ENUM eBufferMode;
OPM402 drvMode;
double dbDistance;
float fVel, fAcceleration,fDeceleration,fJerk;
/* Set sutiable mode for MoveVelocity */
fVel = 100000.0;
axis.MoveVelocity(fVel, eBufferMode);
LOGD << "MoveVelocity" << LEND;
/* Sleep for 2 Sec */
WAIT_SLEEP_MILLI(2000)
LOGD << "STOP" << LEND;
axis.Stop(100000000.0, 1000000000.0,eBufferMode);
WaitFbDone(conn_handle, NC_AXIS_STAND_STILL_MASK, &axis);
LOGD << "Done" << LEND;
/* Retrive the keeping mode */
return;
#endif

  now_pos              = axis.GetActualPosition();
  now_vel              = axis.GetActualVelocity();
  const double pos_err = (target_pos * 1.0 - now_pos);
  const double lim_mA  = get_currentLim();

  const auto v_order = kp * pos_err;
  tor_order          = kd * (v_order - now_vel + tor_order_integral);

  if(torLimFlag == false) {
    tor_order_integral += ki * (v_order - now_vel);
  }

  tor_order = std::min(std::max(tor_order, -lim_mA), lim_mA);
  std::cout << "p " << now_pos << " \t v " << now_vel << " \t t " << target_pos << " \t [tor] " << tor_order << " [lim] " << lim_mA << std::endl;
  axis.MoveTorque(tor_order, 5.0 * pow(10, 6), 1.0 * pow(10, 8), MC_ABORTING_MODE);

  //			single_axis.MoveAbsoluteRepetitive(target_pos,MC_ABORTING_MODE);
  //			cout <<"vOrd" << v_order <<  "tOrd" << tor_order << " ** ";
  // For Abnormal termination
  this->check_status();
}

bool TorControls::init(const std::string& axisName, const MMC_CONNECT_HNDL& gConnHndl) {
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

  std::cout << " initializing axis : " << axisName << std::endl;
  axis.InitAxisData(axisName.c_str(), gConnHndl);
  // axis.SetDefaultParams(stSingleDefault);
  axis.m_fAcceleration = 10000;

  conn_handle = gConnHndl;
  return this->check_status();
}


bool TorControls::poweron() {
  std::cout << "power on start......" << std::endl;
  int ret = axis.SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
  std::cout << "change is successed if ret == 0, and ret is ..." << ret << endl;
  if(ret != 0) {
    LOGE << "Set error" << LEND;
  } else {
    int m = axis.GetOpMode();
    if(m != 10) {
      LOGE << "Axis operation Mode is not 10!! --> " << m << LEND;
      LOGE << "power on exitting...." << LEND;
      return false;
    }
  }

  axis.PowerOn();
  WaitFbDone(conn_handle, NC_AXIS_STAND_STILL_MASK, &axis);
  cout << "power on is completed" << endl;
  return this->check_status();
}

bool TorControls::poweroff() {
  axis.PowerOff();
  std::cout << " power OFF" << std::endl;
  return this->check_status();
}

void TorControls::abort() {
  std::cout << " power off....." << std::endl;
  axis.PowerOff();
  std::cout << " abort called poweroff axis ....." << std::endl;
}


bool TorControls::check_status() {
  int Status = axis.ReadStatus();
  if(Status & NC_AXIS_ERROR_STOP_MASK) {
    axis.Reset();
    LOGE << " AXIS RESET CALLED!! sleeping....." << LEND;
    sleep(1);
    Status = axis.ReadStatus();
    if(Status & NC_AXIS_ERROR_STOP_MASK) {
      LOGE << "Axis is in Error Stop. Aborting." << LEND;
      return false;
    }
    std::cout << "power OFF" << std::endl;
  }
  return true;
}
