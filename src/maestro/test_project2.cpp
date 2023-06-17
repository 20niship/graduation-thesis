#include "test_project2.h" // Application header file.
#include "mmcpplib.h"
#include <MMC_definitions.h>
#include <iostream>
#include <signal.h>   // For Timer mechanism
#include <sys/time.h> // For time structure

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

#include "logger.h"

bool MBUS_PACKET_FLAG = false;
#define MBUS_CONNCETION_SUCESS_LIM 10  // about 0.1sec
#define MBUS_CONNCETION_TIMEOUT_LIM 10 // about 0.1sec

int mbus_timeout_counter;
int fromTorqueControlMode;

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

void MainInit() {
  gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK)CallbackFunc);
  cHost.MbusStartServer(gConnHndl, 1);
  CMMCPPGlobal::Instance()->SetThrowFlag(true, false); // 	Enable throw feature. @ axis
  CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
  cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE, (void*)ModbusWrite_Received);
  cConn.RegisterEventCallback(MMCPP_EMCY, (void*)Emergency_Received);

  memset(mbus_write_in.regArr, 0x0, 250);
  sleep(1);
}

void MainClose() {
  std::cout << "closing modbus server" << std::endl;
  cHost.MbusStopServer();
  std::cout << " closing motor connection" << std::endl;
  MMC_CloseConnection(gConnHndl);
  std::cout << " exiting........" << std::endl;
}

void MachineSequences() {
  MachineSequencesInit();
  EnableMachineSequencesTimer(TIMER_CYCLE);
  while(!giTerminate) {
    MachineSequencesTimer(0);
    BackgroundProcesses();
    usleep(SLEEP_TIME);
  }
}

void MachineSequencesInit() {
  giTerminate  = FALSE;
  giReentrance = FALSE;
}


void BackgroundProcesses() {
  // const auto time   = std::chrono::system_clock::now();
  // auto hour         = std::chrono::duration_cast<std::chrono::hours>(time.time_since_epoch()).count() % 24;
  // auto minute       = std::chrono::duration_cast<std::chrono::minutes>(time.time_since_epoch()).count() % 60;
  // auto seconds      = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() % 60;
  // auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() % 1000;
  // std::cout << "[time] " << hour << ":" << minute << ":" << seconds << ":" << milliseconds << std::endl;
}

void EnableMachineSequencesTimer(int TimerCycle) {
  struct itimerval timer;
  struct sigaction stSigAction;
#if 1
  stSigAction.sa_handler = TerminateApplication;
  sigaction(SIGINT, &stSigAction, NULL);
  sigaction(SIGTERM, &stSigAction, NULL);
  sigaction(SIGABRT, &stSigAction, NULL);
  sigaction(SIGQUIT, &stSigAction, NULL);
  sigaction(SIGKILL, &stSigAction, NULL);
#else
  std::signal(SIGINT, MainClose);
  std::signal(SIGTERM, MainClose);
  std::signal(SIGABRT, MainClose);
  std::signal(SIGQUIT, MainClose);
  std::signal(SIGALRM, MainClose);
  std::signal(SIGKILL, MainClose);
  std::signal(SIGSTOP, MainClose);
  std::signal(SIGTSTP, MainClose);
#endif

  //
  //	Enable the main machine sequences timer function
  //
  timer.it_interval.tv_sec  = 0;
  timer.it_interval.tv_usec = TimerCycle * 1000; // From ms to micro seconds
  timer.it_value.tv_sec     = 0;
  timer.it_value.tv_usec    = TimerCycle * 1000; // From ms to micro seconds

  //	setitimer(ITIMER_REAL, &timer, NULL);			- Temporarily !!!
  //	signal(SIGALRM, MachineSequencesTimer); 		// every TIMER_CYCLE ms SIGALRM is received which calls MachineSequencesTimer()
  return;
}


void MachineSequencesTimer(int iSig) {
  if(giTerminate) return; //	Avoid reentrance of this time function
  if(giReentrance) {
    printf("Reentrancy!\n");
    return;
  }                    //	Error handling should be taken by the user.
  giReentrance = TRUE; // to enable detection of reentrancy.
  ReadAllInputData();  //	Read all input data.
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
      MBUS_PACKET_FLAG = FALSE; // for Re-entrance avoidance
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

  WriteAllOutputData();
  giReentrance = FALSE; //	Clear the reentrancy flag
  return;               // End of the sequences timer function.
}

/*
============================================================================
 Function:				ReadAllInputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 The data used during the processing of the states machines must be synchronized with the
 timer event and must be "frozen" during the processing of all states machines code, so that
 all decisions and calculations will use the same data.

 This function is called at the beginning of the timer function, each TIMER_CYCLE ms,
 to collect all required input data (as relevant to each application) and to copy
 it into "mirror" variables, that will be used by the state machines code.
 ============================================================================
*/
void ReadAllInputData() {
  MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;

  // TODO: Change the number of registers to read.
  cHost.MbusReadHoldingRegisterTable(MODBUS_READ_OUTPUTS_INDEX, MODBUS_READ_CNT, mbus_read_out);
  //
  // TODO: Extract Data read from Modbus.
  //
  /* giXStatus = a1.ReadStatus(); */
  /* giYStatus = a2.ReadStatus(); */
  control_a1.check_status();
  return;
}

void WriteAllOutputData() {
  //
  //	Here should come the code to write/send all ouput data
  //
  mbus_write_in.startRef = MODBUS_UPDATE_START_INDEX; // index of start write modbus register.
  mbus_write_in.refCnt   = MODBUS_UPDATE_CNT;         // number of indexes to write
  //
  // TODO: Prepare Data to be writen to Modbus.
  //
  cHost.MbusWriteHoldingRegisterTable(mbus_write_in);
  return;
}


int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock) {
  std::cout << "callback called" << std::endl;
  // Which function ID was received ...
  switch(recvBuffer[1]) {
    case EMCY_EVT:
      //
      // Please note - The emergency event was registered.
      // printf("Emergency Event received\r\n") ;
      break;
    case MOTIONENDED_EVT: printf("Motion Ended Event received\r\n"); break;
    case HBEAT_EVT: printf("H Beat Fail Event received\r\n"); break;
    case PDORCV_EVT: printf("PDO Received Event received - Updating Inputs\r\n"); break;
    case DRVERROR_EVT: printf("Drive Error Received Event received\r\n"); break;
    case HOME_ENDED_EVT: printf("Home Ended Event received\r\n"); break;
    case SYSTEMERROR_EVT: printf("System Error Event received\r\n"); break;
    /* This is commented as a specific event was written for this function. Once it occurs
     * the ModbusWrite_Received will be called
      case MODBUS_WRITE_EVT:
      // TODO Update additional data to be read such as function parameters.
      // TODO Remove return 0 if you want to handle as part of callback.
      return 0;
      printf("Modbus Write Event received - Updating Outputs\r\n") ;

      break ;
    */
    default: std::cout << " hoghoge" << std::endl;
  }
  //
  return 1;
}

int OnRunTimeError(const char* msg, unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) {
  LOGE << "MMCPPExitClbk: Run time Error in function " << msg << ", axis ref=" << usAxisRef << ", err=" << sErrorID << ", status=" << usStatus << ", bye\n" << LEND;
  TerminateApplication(0);
  // exit(0);
  return 0;
}

void TerminateApplication(int iSigNum) {
  if(giTerminate == 1) {
    LOGE << "TerminateApplicaiton関数が複数回呼ばれたのでexitします...." << LEND;
    exit(0);
  }
  LOGW << "TerminateApplicaiton関数が呼ばれました...." << LEND;
  giTerminate = 1;
  MainClose();
  control_a1.abort();
  sigignore(SIGALRM);
  sleep(1);
}

void ModbusWrite_Received() {
  MBUS_PACKET_FLAG = TRUE;
  printf("Modbus Write Received\n");
}

void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) { printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode); }
