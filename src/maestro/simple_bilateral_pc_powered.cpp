#include "common.hpp"
#include "src/hr4c.hpp"
#include <MMC_definitions.h>
#include <iostream>
#include <semaphore.h>

TorControls control_a1, control_a2;

int main(int argc, char* argv[]) {
  float kp = 0.01;
  float kd = 1;

  if(argc == 3) {
    kp = std::stof(argv[1]);
    kd = std::stof(argv[2]);
  }

  init_logger();
  spdlog::info("kp: {} kd: {}", kp, kd);
  try {
    MainInit();
    control_a1 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);
    control_a2 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);

    if(!control_a1.init("a01", gConnHndl)) {
      spdlog::error("torque control init failed");
      goto terminate;
    }

    if(!control_a2.init("a02", gConnHndl)) {
      spdlog::error("torque control init failed");
      goto terminate;
    }

    std::cout << "torque control poweron" << std::endl;
    if(!control_a1.poweron()) {
      spdlog::error("torque control 1 poweron failed");
      goto terminate;
    }
    if(!control_a2.poweron()) {
      spdlog::error("torque control 2 poweron failed");
      goto terminate;
    }

    control_a1.set_KP(kp);
    control_a1.set_KD(kd);

    control_a2.set_KP(kp);
    control_a2.set_KD(kd);

    StartMain();
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
  assert(sizeof(T) % sizeof(int16_t) == 0);
  assert(index < MODBUS_WRITE_IN_CNT);
  const void* p = static_cast<const void*>(&arg_value);
  for(size_t i = 0; i < sizeof(T) / sizeof(int16_t); i++) ptr[index + i] = ((int16_t*)p)[i];
};

template <typename T> T read_n_from16bit(const int16_t* ptr, int index) {
  assert(sizeof(T) % sizeof(int16_t) == 0);
  assert(index < MODBUS_READ_CNT);
  T ret;
  void* p = static_cast<void*>(&ret);
  for(size_t i = 0; i < sizeof(T) / sizeof(int16_t); i++) ((int16_t*)p)[i] = ptr[index + i];
  return ret;
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
    int32_t tmp_tor = control_a1.get_tor_order();
    int start_ref   = hr4c::eAx1;
    send_n_to16bit<int32_t>(tmp_pos, mbus_write_in.regArr, start_ref + hr4c::eActualPos);
    send_n_to16bit<int32_t>(tmp_vel, mbus_write_in.regArr, start_ref + hr4c::eActualVel);
    send_n_to16bit<int32_t>(tmp_tor, mbus_write_in.regArr, start_ref + hr4c::eActualTor);
    // spdlog::info("pos: {}, vel: {}, tor: {}", tmp_pos, tmp_vel, tmp_tor);

    start_ref = hr4c::eAx2;
    tmp_pos   = control_a2.get_pos();
    tmp_vel   = control_a2.get_vel();
    tmp_tor   = control_a2.get_tor_order();
    send_n_to16bit<int32_t>(tmp_pos, mbus_write_in.regArr, start_ref + hr4c::eActualPos);
    send_n_to16bit<int32_t>(tmp_vel, mbus_write_in.regArr, start_ref + hr4c::eActualVel);
    send_n_to16bit<int32_t>(tmp_tor, mbus_write_in.regArr, start_ref + hr4c::eActualTor);
    // spdlog::info("pos: {}, vel: {}, tor: {}", tmp_pos, tmp_vel, tmp_tor);
  }

  if(isKeyPressed()) {
    spdlog::info("Terminate app by key press");
    TerminateApplication(0);
  }
}

void ModbusWrite_Received() {
  spdlog::info("Modbus Write Received");
  auto hoge   = read_n_from16bit<int32_t>(mbus_read_out.regArr, hr4c::eCommand1);
  auto target = read_n_from16bit<double>(mbus_read_out.regArr, hr4c::eCommand2);
  if(hoge != 0 && target != 0) {
    spdlog::info("move to {}, {}", hoge, target);
    control_a2.set_target(target);
  }
  for(int i = 0; i < MODBUS_READ_CNT; i++) mbus_read_out.regArr[i] = 0;
}
