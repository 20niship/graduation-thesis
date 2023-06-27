#pragma once

#include <pthread.h>
#include <semaphore.h>

#include <MMCConnection.h>
#include <MMCSingleAxis.h>
#include <MMC_definitions.h>
#include <MMC_host_comm_API.h>

#include "TorqueControl.hpp"

/*
============================================================================
 Name : CPP_Template.h
 Author  :		Benjamin Spitzer
 Version :
 Description : 	GMAS C/C++ project header file for Template
============================================================================
*/

/*
============================================================================
 Project general functions prototypes
============================================================================
*/
void MainInit();
void MachineSequences();
void MainClose();
void MachineSequencesInit();
void EnableMachineSequencesTimer(int TimerCycle);
void BackgroundProcesses();
void MachineSequencesClose();
void MachineSequencesTimer(int iSig);
void ReadAllInputData();
void WriteAllOutputData();
int OnRunTimeError(const char* msg, unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
void TerminateApplication(int iSigNum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);
void ModbusWrite_Received();
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock);
/*
============================================================================
 States functions
============================================================================
*/
void StateFunction_1(); // TODO: Change the name of these functions accordingly
void SubState1_1Function();
void SubState1_2Function();
void SubState1_3Function();
void SubState1_4Function();

void StateFunction_2(); // TODO: Change the name of these functions accordingly
void SubState2_1Function();
void SubState2_2Function();

void StateXYDefaultFunction();
/*
============================================================================
 General constants
============================================================================
*/
#define MAX_AXES 2 // number of Physical axes in the system. TODO Update MAX_AXES accordingly
#define FALSE 0
#define TRUE 1
//
// TODO: Modbus memory map offsets must be updated accordingly.
#define MODBUS_READ_OUTPUTS_INDEX 0  // Start of Modbus read address
#define MODBUS_READ_CNT 16           // Number of registers to read
#define MODBUS_UPDATE_START_INDEX 16 // Start of Modbus write address (update to host)
#define MODBUS_UPDATE_CNT 16         // Number of registers to update
/*
============================================================================
 Project constants
============================================================================
*/
#define SLEEP_TIME 10000 // Sleep time of the backround idle loop, in micro seconds
#define TIMER_CYCLE 20   // Cycle time of the main sequences timer, in ms
/*
============================================================================
 States Machines constants
============================================================================
*/
#define FIRST_SUB_STATE 1
enum eMainStateMachines // TODO: Change names of state machines to reflect dedicated project
{
  eIDLE = 0,
  eSM1  = 1, // Main state machine #1
};

enum eSubStateMachine_1 // TODO: Change names of sub-state machines.
{
  eSubState_SM1_1 = 1,
  eSubState_SM1_2 = 2,
  eSubState_SM1_3 = 3,
  eSubState_SM1_4 = 4,
};
enum eSubStateMachine_2 // TODO: Change names of sub-state machines.
{
  eSubState_SM2_1 = 1,
  eSubState_SM2_2 = 2,
  eSubState_SM2_3 = 3,
  eSubState_SM2_4 = 4,
};

/*
============================================================================
 Application global variables
============================================================================
*/
int giTerminate;  // Flag to request program termination
int giReentrance; // Used to detect reentrancy to the main timer function
int giPrevState1;
int giState1;
/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL gConnHndl; // Connection Handle
CMMCConnection cConn;
CMMCHostComm cHost;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT mbus_write_out;

TorControls control_a1;
