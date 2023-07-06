#include "mmcpplib.h"
#include "src/TorqueControl.hpp"
#include "src/common.h"
#include "src/get_cmmc_exception_error_message.hpp"

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

#include "common.hpp"
#include "src/logger.h"
#include <semaphore.h>

#include <iostream>

TorControls control_a1, control_a2;

int main(int argc, char* argv[]) {
  if(argc != 3) {
    std::cout << "Usage: " << argv[0] << " <kp> <kd>" << std::endl;
    return 0;
  }

  float kp = std::stof(argv[1]);
  float kd = std::stof(argv[2]);

  init_logger();
  spdlog::info("kp: {} kd: {}", kp, kd);
  try {
    MainInit();
    control_a1 = TorControls(166, 0.095, 1.5, 2000);
    control_a2 = TorControls(166, 0.095, 1.5, 2000);

    if(!control_a1.init("a01", gConnHndl)) {
      spdlog::error("torque control init failed");
      goto terminate;
    }

    if(!control_a2.init("a02", gConnHndl)) {
      spdlog::error("torque control init failed");
      goto terminate;
    }

    std::cout << "torque control poweron" << std::endl;

    MachineSequences();
    return 0;
  } catch(CMMCException excp) {
    spdlog::error("CMMCException: {}", excp.what());
    spdlog::error("   : axisref = {}", excp.axisRef());
    spdlog::error("   : error = {}", excp.error());
    // Maestro Administrative and Motion API.pdf の 66ページ参照
    spdlog::error("   : status = {}", excp.status());

    const auto msg = get_cmmc_exception_error_message(excp);
    spdlog::error(msg);

    DISP(excp.what());
    DISP(excp.axisRef());
    DISP(excp.error());
    DISP(excp.status());

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
  giTerminate = true;
  spdlog::info("MainClose end");
  return 1;
}

void terminateApp() {
  control_a1.abort();
  control_a2.abort();
}

void update() {
  if(giTerminate) return; //	Avoid reentrance of this time function
  static unsigned long nFrames = 0;
  nFrames++;

  control_a2.update();
  auto pos = control_a2.get_pos();
  control_a1.set_target(pos);
  control_a1.p_pi_controlAxis();

  if(isKeyPressed()) {
    spdlog::info("Terminate app by key press");
    TerminateApplication(0);
  }
}
