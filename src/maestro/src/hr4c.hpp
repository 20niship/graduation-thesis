#include <MMC_definitions.h>
#include <iostream>
#include <pthread.h>
#include <signal.h>   // For Timer mechanism
#include <sys/time.h> // For time structure

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

#include "logger.h"
#include <semaphore.h>

#include <MMCConnection.h>
#include <MMCSingleAxis.h>
#include <MMC_definitions.h>
#include <MMC_host_comm_API.h>

#include "TorqueControl.hpp"
#include "get_cmmc_exception_error_message.hpp"
#include "../../common/communication_const.hpp"

void MainInit();
void StartMain();
void MainClose();
void TerminateApplication(int iSigNum);

extern bool giTerminate;  // Flag to request program termination
extern MMC_CONNECT_HNDL gConnHndl; // Connection Handle
extern CMMCConnection cConn;
extern CMMCHostComm cHost;
extern MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;
extern MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;

// --- userが定義する必要がある関数 ---
void update();
void terminateApp();
void ModbusWrite_Received();

