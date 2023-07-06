#include "src/TorqueControl.hpp"
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
#include "src/hr4c.hpp"
#include "src/logger.h"
#include <semaphore.h>

#include <iostream>

TorControls control_a1, control_a2;

int main(int argc, char* argv[]) {
  init_logger();
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

template <typename T> void send_n_to16bit(const T arg_value, int16_t* ptr, int index) {
  const void* p = static_cast<const void*>(&arg_value);
  for(size_t i = 0; i < sizeof(T) / sizeof(int16_t); i++) ptr[index + i] = ((int16_t*)p)[i];
};

void update() {
  if(giTerminate) return; //	Avoid reentrance of this time function
  static unsigned long nFrames = 0;
  nFrames++;

  control_a2.update();
  auto pos = control_a2.get_pos();
  control_a1.set_target(pos);
  control_a1.p_pi_controlAxis();

  {
    int32_t tmp_pos = control_a1.get_pos();
    int32_t tmp_vel = control_a1.get_vel();
    int32_t tmp_tor = control_a1.get_tor_order(); // mA
    int start_ref   = hr4c::eAx1;
    send_n_to16bit<int32_t>(tmp_pos, mbus_write_in.regArr, start_ref);
    send_n_to16bit<int32_t>(tmp_vel, mbus_write_in.regArr, start_ref + 2);
    send_n_to16bit<int32_t>(tmp_tor, mbus_write_in.regArr, start_ref + 4);

    start_ref   = hr4c::eAx2;
  }

  if(isKeyPressed()) {
    spdlog::info("Terminate app by key press");
    TerminateApplication(0);
  }
}
