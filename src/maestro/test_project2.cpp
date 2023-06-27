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
  LOGD << "maininit start" << LEND;
  MainInit();
  LOGD << "maininit end" << LEND;

  if(argc < 1) {
    LOGE << "Usage: " << argv[0] << " <axis>" << LEND;
    return 1;
  }
  const char* axis_name = argv[1];

  // default is 240, 1.85*pow(10,-5), 29.7 1/10 is good?
  control_a1 = TorControls(166, 0.095 * std::pow(10, -5), 1.5, 2000);
  std::cout << "torque control init" << std::endl;
  control_a1.init(axis_name, gConnHndl);
  std::cout << "torque control poweron" << std::endl;
  control_a1.poweron();

  LOGD << "MachineSequences start" << LEND;
  MachineSequences();
  LOGD << "MachineSequences end" << LEND;

  MainClose();
  LOGD << "MainClose end" << LEND;
  return 1;
}

void MainInit() {
  LOGI << 1 << LEND;
  gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK)CallbackFunc);
  LOGI << "hoge" << LEND;
  cHost.MbusStartServer(gConnHndl, 1);

  LOGI << 15 << LEND;
  CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
  LOGI << 6 << LEND;

  cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE, (void*)ModbusWrite_Received);
  cConn.RegisterEventCallback(MMCPP_EMCY, (void*)Emergency_Received);

  memset(mbus_write_in.regArr, 0x0, 250);
  LOGI << 10 << LEND;
  return;
}

void MainClose() {
  cHost.MbusStopServer();
  MMC_CloseConnection(gConnHndl);
  return;
}

void MachineSequences() {
  MachineSequencesInit();
  EnableMachineSequencesTimer(TIMER_CYCLE);
  while(!giTerminate) {
    MachineSequencesTimer(0);
    BackgroundProcesses();
    usleep(SLEEP_TIME);
  }
  MachineSequencesClose();
  return;
}

void MachineSequencesInit() {
  giTerminate  = FALSE;
  giReentrance = FALSE;
  return;
}

void MachineSequencesClose() { return; }

void BackgroundProcesses() {
  const auto time   = std::chrono::system_clock::now();
  auto hour         = std::chrono::duration_cast<std::chrono::hours>(time.time_since_epoch()).count() % 24;
  auto minute       = std::chrono::duration_cast<std::chrono::minutes>(time.time_since_epoch()).count() % 60;
  auto seconds      = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() % 60;
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() % 1000;
  std::cout << "[time] " << hour << ":" << minute << ":" << seconds << ":" << milliseconds << std::endl;
  return;
}

void EnableMachineSequencesTimer(int TimerCycle) {
  struct itimerval timer;
  struct sigaction stSigAction;

  // Whenever a signal is caught, call TerminateApplication function
  stSigAction.sa_handler = TerminateApplication;

  sigaction(SIGINT, &stSigAction, NULL);
  sigaction(SIGTERM, &stSigAction, NULL);
  sigaction(SIGABRT, &stSigAction, NULL);
  sigaction(SIGQUIT, &stSigAction, NULL);
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
/*
============================================================================
 Function:				MachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A timer function that is called by the OS every TIMER_CYCLE ms.
 It executes the machine sequences states machines and actully controls
 the sequences and behavior of the machine.
============================================================================
*/
void MachineSequencesTimer(int iSig) {
  if(giTerminate == TRUE) return; //	Avoid reentrance of this time function
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
  //
  //	Here should come the code to read all required input data, for instance:
  //
  //	giXInMotion = ...
  //	giYInMotion = ...
  //
  // The data read here may arrive from different sources:
  // 	- Host Communication (Modbus, Ethernet-IP. This can be read on a cyclic basis, or from a callback.
  //	- GMAS Firmware. Such as actual positions, torque, velocities.

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
/*
============================================================================
 Function:				WriteAllOutputData()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Write all the data, generated by the states machines code, to the "external
 world".

 This is done here (and not inside the states machines code) to ensure that
 all data is updated simultaneously and in a synchronous way.

 The states machines code write the data into "mirror" variables, that are here
 copied or sent to the "external world" (Host via MODBUS, GMAS core firmware, etc.)
  ============================================================================
*/
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interupt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
  MMC_CloseConnection(uiConnHndl);
  printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
  exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void TerminateApplication(int iSigNum) {
  printf("In Terminate Application ...\n");
  giTerminate = 1;
  sigignore(SIGALRM);
  sleep(1);
  return;
}

//
// Callback Function once a Modbus message is received.
void ModbusWrite_Received() {
  MBUS_PACKET_FLAG = TRUE;
  printf("Modbus Write Received\n");
}

//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) { printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode); }
