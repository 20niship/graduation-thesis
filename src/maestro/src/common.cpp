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

bool giTerminate = false; // Flag to request program termination

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
  sigaction(SIGSTOP, &stSigAction, NULL);
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
  spdlog::error("CallbackFunc called");
  spdlog::error("  recvBufferSize = {}", recvBufferSize);
  spdlog::error("   Event  type = {}", (int)recvBuffer[1]);
  for(size_t i = 0; i < recvBufferSize; i++) spdlog::warn(" buf[{}] = {}", i, (int)recvBuffer[i]);
  spdlog::error("  lpsok = {}", lpsock);

  std::string error_msg = "unknown error";
  switch(recvBuffer[1]) {
    case EMCY_EVT: error_msg = "Emergency Event Recieeved"; break;
    case MOTIONENDED_EVT: error_msg = "Motion Ended Event received"; break;
    case HBEAT_EVT: error_msg = "H Beat Fail Event received"; break;
    case PDORCV_EVT: error_msg = "PDO Received Event received - Updating Inputs"; break;
    case DRVERROR_EVT: error_msg = "Drive Error Received Event received"; break;
    case HOME_ENDED_EVT: error_msg = "Home Ended Event received"; break;
    case SYSTEMERROR_EVT: error_msg = "System Error Event received"; break;
    case MODBUS_WRITE_EVT: error_msg = "Modbus Write Event received - Updating Outputs"; break;
    case TOUCH_PROBE_ENDED_EVT: error_msg = "Touch Probe Ended Event received"; break;
    case NODE_ERROR_EVT: error_msg = "Node Error Event received"; break;
    case STOP_ON_LIMIT_EVT: error_msg = "Stop On Limit Event received"; break;
    case TABLE_UNDERFLOW_EVT: error_msg = "Table Underflow Event received"; break;
    case NODE_CONNECTED_EVT: error_msg = "Node Connected Event received"; break;
    case GLOBAL_ASYNC_REPLY_EVT: error_msg = "Global Async Reply Event received"; break;
    case NODE_INIT_FINISHED_EVT: error_msg = "Node Init Finished Event received"; break;
    case FB_NOTIFICATION_EVT: error_msg = "FB Notification Event received"; break;
    case POLICY_ENDED_EVT: error_msg = "Policy Ended Event received"; break;
    default: error_msg = "unknown error";
  }
  spdlog::error("  error_msg = {}", error_msg);
  giTerminate = true;
  return 1;
}

int OnRunTimeError(const char* msg, unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) {
  spdlog::error("MMCPPExitClbk: Run time Error in function {}, axis ref={}, err={}, status={}, bye", msg, usAxisRef, sErrorID, usStatus);
  TerminateApplication(0);
  // exit(0);
  return 0;
}


void TerminateApplication(int iSigNum) {
  if(giTerminate) {
    spdlog::error("TerminateApplicaiton関数が複数回呼ばれたのでexitします....");
    exit(0);
  }
  spdlog::warn("TerminateApplicaiton called exiting");
  sigignore(SIGALRM);
  terminateApp();
  MainClose();
  // control_a1.abort();
  exit(0);
  giTerminate = true;
}

void ModbusWrite_Received() {
  MBUS_PACKET_FLAG = TRUE;
  spdlog::info("Modbus Write Received");
}

void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) { 
  spdlog::error("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}

void MachineSequencesTimer(int iSig) {
  if(giTerminate) return; //	Avoid reentrance of this time function
  ReadAllInputData();
  update();
  WriteAllOutputData();
  return;
}
