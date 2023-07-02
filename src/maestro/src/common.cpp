#include "common.h"

#define FALSE 0
#define TRUE 1
//
// TODO: Modbus memory map offsets must be updated accordingly.
#define MODBUS_READ_OUTPUTS_INDEX 0  // Start of Modbus read address
#define MODBUS_READ_CNT 16           // Number of registers to read
#define MODBUS_UPDATE_START_INDEX 16 // Start of Modbus write address (update to host)
#define MODBUS_UPDATE_CNT 16         // Number of registers to update
#define SLEEP_TIME 10000             // Sleep time of the backround idle loop, in micro seconds
#define TIMER_CYCLE 20               // Cycle time of the main sequences timer, in ms

#define FIRST_SUB_STATE 1

bool giTerminate=false;  // Flag to request program termination

MMC_CONNECT_HNDL gConnHndl; // Connection Handle
CMMCConnection cConn;
CMMCHostComm cHost;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT mbus_write_out;

bool MBUS_PACKET_FLAG = false;
#define MBUS_CONNCETION_SUCESS_LIM 10  // about 0.1sec
#define MBUS_CONNCETION_TIMEOUT_LIM 10 // about 0.1sec

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
  EnableMachineSequencesTimer(TIMER_CYCLE);
  while(!giTerminate) {
    MachineSequencesTimer(0);
    BackgroundProcesses();
    usleep(SLEEP_TIME);
  }
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


void ReadAllInputData() {
  MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;
  cHost.MbusReadHoldingRegisterTable(MODBUS_READ_OUTPUTS_INDEX, MODBUS_READ_CNT, mbus_read_out);
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
  if(giTerminate) {
    LOGE << "TerminateApplicaiton関数が複数回呼ばれたのでexitします...." << LEND;
    exit(0);
  }
  LOGW << "TerminateApplicaiton関数が呼ばれました...." << LEND;
  MainClose();

  terminateApp();

  // control_a1.abort();
  sigignore(SIGALRM);
  sleep(1);
}

void ModbusWrite_Received() {
  MBUS_PACKET_FLAG = TRUE;
  printf("Modbus Write Received\n");
}

void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) { printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode); }


void MachineSequencesTimer(int iSig) {
  if(giTerminate) return; //	Avoid reentrance of this time function
  ReadAllInputData();  
  update();
  WriteAllOutputData();
  return;              
}
