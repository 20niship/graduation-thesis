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

#include "logger.h"
#include <semaphore.h>

#define FIRST_SUB_STATE 1
enum eMainStateMachines // TODO: Change names of state machines to reflect dedicated project
{
  eIDLE = 0,
  eSM1  = 1, // Main state machine #1
};

int giPrevState1;
int giState1;
TorControls control_a1;

#define MBUS_CONNCETION_SUCESS_LIM 10  // about 0.1sec
#define MBUS_CONNCETION_TIMEOUT_LIM 10 // about 0.1sec

int main(int argc, char* argv[]) {
  if(argc < 2) {
    LOGE << "Usage: " << argv[0] << " <axis>" << LEND;
    return 1;
  }
  const char* axis_name = argv[1];

  try {
    MainInit();

    control_a1 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);
    std::cout << "torque control init  axis= [" << axis_name << "]" << std::endl;
    bool ret = control_a1.init(axis_name, gConnHndl);
    if(!ret) {
      LOGE << "torque control init failed" << LEND;
      goto terminate;
    }
    std::cout << "torque control poweron" << std::endl;
    ret = control_a1.poweron();
    if(!ret) {
      LOGE << "torque control poweron failed" << LEND;
      goto terminate;
    }
    MachineSequences();
  } catch(CMMCException excp) {
    LOGE << "CMMCException: " << excp.what() << LEND;
    LOGE << "   : axisref = " << excp.axisRef() << LEND;
    LOGE << "   : error = " << excp.error() << LEND;
    LOGE << "   : status = " << excp.status() << LEND;
    goto terminate;
  } catch(std::exception& e) {
    LOGE << "std Exception: " << e.what() << LEND;
    goto terminate;
  } catch(...) {
    LOGE << "Unknown exception" << LEND;
    goto terminate;
  }

terminate:
  MainClose();
  LOGD << "MainClose end" << LEND;
  return 1;
}

void terminateApp() { control_a1.abort(); }

void update() {
  if(giTerminate) return; //	Avoid reentrance of this time function
  if(giReentrance) {
    printf("Reentrancy!\n");
    return;
  }
  giPrevState1 = giState1;

  switch(giState1) {
    case eIDLE: // Recieve the packet of modbus
    {
      giState1 = eSM1;
      break;

      if(MBUS_PACKET_FLAG) {
        static int counter = 0;
        counter += 1;
        if(counter > MBUS_CONNCETION_SUCESS_LIM) {
          giState1 = eSM1;
        }
      } else {
        static int counter2 = 0;
        std::cout << "waiting for host connection......." << std::endl;
        counter2 += 1;
        if(counter2 > MBUS_CONNCETION_TIMEOUT_LIM) {
          LOGW << "hostPCが認識できなかったので次に進みます" << LEND;
          // Treat Connection false as " hostPC doesn't send signals "
          control_a1.reset_integral();
          giState1 = eSM1;
          break;
        }
      }
      MBUS_PACKET_FLAG = false; // for Re-entrance avoidance
      break;
    }

    case eSM1: {
      control_a1.p_pi_controlAxis();
      giState1 = eIDLE;
      break;
    }

    default: // The default case. Should not happen, the user can implement error handling.
    {
      std::cout << "<<default>>" << std::endl;
      giState1 = eIDLE;
      break;
    }
  }
  giReentrance = false;
  return;
}
