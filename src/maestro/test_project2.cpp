#include "mmcpplib.h"
#include "src/TorqueControl.hpp"
#include "src/common.h"

#include <MMC_definitions.h>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

#include "src/logger.h"
#include <semaphore.h>

#define FIRST_SUB_STATE 1
enum eMainStateMachines {
  eIDLE = 0,
  eSM1  = 1,
};

int giPrevState1;
int giState1;
TorControls control_a1, control_a2;

int main(int argc, char* argv[]) {
  if(argc != 3) {
    std::cout << "Usage: " << argv[0] << " <kp> <kd>" << std::endl;
    return 0;
  }

  float kp = std::stof(argv[1]);
  float kd = std::stof(argv[2]);
  std::cout << "kp: " << kp << " kd: " << kd << std::endl;

  try {
    MainInit();
    control_a1 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);
    control_a2 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);

    if(!control_a1.init("a01", gConnHndl)){
      spdlog::error("torque control init failed");
      goto terminate;
    }

    if(!control_a2.init("a02", gConnHndl)){
      spdlog::error("torque control init failed");
      goto terminate;
    }


    std::cout << "torque control poweron" << std::endl;
    if(!control_a1.poweron()) {
      spdlog::error("torque control poweron failed");
      goto terminate;
    }
    if(!control_a2.poweron()){
      spdlog::error("torque control poweron failed");
      goto terminate;
    }

    MachineSequences();
    return 0;
  } catch(CMMCException excp) {
    spdlog::error("CMMCException: {}", excp.what());
    spdlog::error("   : axisref = {}", excp.axisRef());
    spdlog::error("   : error = {}", excp.error());
    spdlog::error("   : status = {}", excp.status());
    goto terminate;
  } catch(std::exception& e) {
    spdlog::error("std Exception: {}", e.what());
    goto terminate;
  } catch(...) {
    spdlog::error("unknown exception");
    goto terminate;
  }

terminate:
  MainClose();
  giTerminate=true;
  spdlog::info("MainClose end");
  return 1;
}

void terminateApp() { control_a1.abort(); control_a2.abort();}

void update() {
  if(giTerminate) return; //	Avoid reentrance of this time function
  static unsigned long nFrames = 0;
  nFrames++;

  control_a2.update();
  auto pos = control_a2.get_pos();
  control_a1.set_target(pos);
  control_a1.p_pi_controlAxis();
}
