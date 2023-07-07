#include <algorithm>
#include <math.h>

#include "TorqueControl.hpp"
#include "logger.h"

#ifdef WIN32
#define WAIT_SLEEP_MILLI(WAIT_MILLI_SEC) Sleep(WAIT_MILLI_SEC);
#else
#define WAIT_SLEEP_MILLI(WAIT_MILLI_SEC) usleep(WAIT_MILLI_SEC * 1000);
#endif

#if 0
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
#endif

TorControls::TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard) {
  // TODO Auto-generated constructor stub
  kp                 = kp_pos / (2.0 * M_PI); // [rad/s] to [/s]
  kd                 = kp_vel * 1000.;        // [A/(cnt/s)] to [mA/(cnt/s)]
  ki                 = 0;                     // ki_vel / 1000.;        // [Hz] = [1/s] to [1/ms]
  tor_order_integral = 0;
  torLimFlag         = false;
}

void TorControls::reset_integral() {
  tor_order_integral = 0;
  tor_order          = 0;
  torLimFlag         = false;
}

void TorControls::sync_state() {
  now_pos = axis.GetActualPosition();
  now_vel = axis.GetActualVelocity();
}

void TorControls::p_pi_controlAxis(bool verbose) {
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
  this->update();

  pos_error           = (target_pos - now_pos);
  const double lim_mA = get_currentLim();

  const auto up = kp * pos_error;
  const auto ud = kd * (pos_error - last_pos_error);
  // const auto ui = ki * tor_order_integral;
  // if(torLimFlag == false) {
  //   tor_order_integral += ki * (v_order - now_vel);
  // }

  tor_order = up + ud; //+ ui;
  tor_order = std::min(std::max(tor_order, -lim_mA), lim_mA);
  if(verbose) spdlog::info("p {} \t v {} \t t {} \t [tor] {} [lim] {}", now_pos, now_vel, target_pos, tor_order, lim_mA);

  axis.MoveTorque(tor_order, 5.0e6, 1.0 * 1e8, MC_ABORTING_MODE);

  last_pos_error = pos_error;

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
  m_axisName                    = axisName;

  std::cout << " initializing axis : " << axisName << std::endl;
  axis.InitAxisData(axisName.c_str(), gConnHndl);
  // axis.SetDefaultParams(stSingleDefault);
  axis.m_fAcceleration = 10000;

  conn_handle = gConnHndl;
  return this->check_status();
}


bool TorControls::poweron() {
  std::cout << "power on start......" << m_axisName << std::endl;
  int ret = axis.SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
  if(ret != 0) {
    spdlog::error("set operation mode failed!!");
  } else {
    this->check_status();
    int m = axis.GetOpMode();
    if(m != OPM402_CYCLIC_SYNC_TORQUE_MODE) {
      spdlog::warn("Axis operation Mode is different!! ");
      spdlog::warn(" current mode = {}", m);
      spdlog::warn(" requested mode = {}", (int)OPM402_CYCLIC_SYNC_TORQUE_MODE);
      spdlog::warn("power on exitting....");
      return false;
    }
  }

  axis.PowerOn();
  // WaitFbDone(conn_handle, NC_AXIS_STAND_STILL_MASK, &axis);
  // spdlog::info("power on is completed");
  is_power_on = true;
  return this->check_status();
}

bool TorControls::poweroff() {
  axis.PowerOff();
  spdlog::info("power off axis={}", axis.GetName());
  is_power_on = false;
  return this->check_status();
}

void TorControls::abort() {
  spdlog::info("stop axis={}", m_axisName);
  if(is_power_on) {
    axis.Stop();
    spdlog::info("poweroff axis={}", m_axisName);
    axis.PowerOff();
  }
  spdlog::info("abort axis={}", m_axisName);
}

void TorControls::goto_home() {
  spdlog::info("homing....");
  axis.MoveAbsolute(0, 1000000, 1000000, MC_BUFFERED_MODE);
}

bool TorControls::check_status() {
  int Status = axis.ReadStatus();
  if(Status & NC_AXIS_ERROR_STOP_MASK) {
    axis.Reset();
    spdlog::error("axis reset called!! sleeping.....");
    sleep(1);
    Status = axis.ReadStatus();
    if(Status & NC_AXIS_ERROR_STOP_MASK) {
      spdlog::error("Axis is in Error Stop. Aborting.");
      return false;
    }
  }
  return true;
}
