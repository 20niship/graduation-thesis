/*
============================================================================
 Name : 	test_project2.cpp
 Author :	Benjamin Spitzer
 Version :	1.00
 Description : The following example supports the following functionalities:

The following features are demonstrated for a Ethercat network (Assuming PDOs were initalized via Ethercat configurator):

- Two separate  state machines for handling parallel motions / sequences.
- Modbus callback registration.
- Emergency callback registration.
- Empty modbus reading area.

 The program works with 2 axes - a01 and a02.

============================================================================
*/
#include <MMC_definitions.h>
#include "mmcpplib.h"
#include "test_project2.h"		// Application header file.
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism

#include <chrono>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>

#include "logger.h"

// State Index (int type)
#define 	FIRST_SUB_STATE			1
bool MBUS_PACKET_FLAG = false;
#define		MBUS_CONNCETION_SUCESS_LIM 10	//about 0.1sec
int			mbus_connection_counter;
#define		MBUS_CONNCETION_TIMEOUT_LIM 10	//about 0.1sec

int			mbus_timeout_counter;
int 		fromTorqueControlMode;


/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 The main function of this sample project.
============================================================================
*/

int main()
{
	//
	//	Initialize system, axes and all needed initializations
	//
	LOGD << "maininit start" << LEND;
	MainInit();
	LOGD << "maininit end" << LEND;

	//
	//	Execute the state machine to handle the system sequences and control
	//
	LOGD << "MachineSequences start" << LEND;
	MachineSequences();
	LOGD << "MachineSequences end" << LEND;
	//
	//	Close what needs to be closed before program termination
	//
	LOGD << "MainClose start" << LEND;

	MainClose();
	LOGD << "MainClose end" << LEND;

	//
LOGE << "end exit!!" << LEND;
	return 1;		// Terminate the application program back to the Operating System
}
/*
============================================================================
 Function:				MainInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the system, including axes, communication, etc.
============================================================================
*/
void MainInit()
{
//
//	Here will come initialization code for all system, axes, communication etc.
//
// 	InitializeCommunication to the GMAS:
//
	LOGI <<1<< LEND;
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	LOGI << "hoge" << LEND;
	//
	// Start the Modbus Server:
	cHost.MbusStartServer(gConnHndl,1) ;

	LOGI <<  15 << LEND;
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	LOGI <<  6 << LEND;

	// Register the callback function for Modbus and Emergency:
	cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE,(void*)ModbusWrite_Received) ;
	cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ;
//
// Initialize default parameters. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity	= 0 ;
	stSingleDefault.dbDistance 		= 100000 ;
	stSingleDefault.dbPosition 		= 0 ;
	stSingleDefault.fVelocity 		= 100000 ;
	stSingleDefault.fAcceleration 	= 1000000 ;
	stSingleDefault.fDeceleration 	= 1000000 ;
	stSingleDefault.fJerk 			= 20000000 ;
	stSingleDefault.eDirection 		= MC_POSITIVE_DIRECTION ;
	stSingleDefault.eBufferMode 	= MC_BUFFERED_MODE ;
	stSingleDefault.ucExecute 		= 1 ;
//
// 	TODO: Update number of necessary axes:
//
	a1.InitAxisData("a01",gConnHndl) ;
	a2.InitAxisData("a02",gConnHndl) ;
	//
	// Set default motion parameters. TODO: Update for all axes.
	LOGI <<  7 << LEND;
	a1.SetDefaultParams(stSingleDefault) ;
	a1.SetDefaultParams(stSingleDefault) ;
	//
	// You may of course change internal parameters manually:
	LOGI <<  8 << LEND;
	a1.m_fAcceleration=10000;
	giXStatus 	= a1.ReadStatus() ;
	if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a1.Reset() ;
		sleep(1) ;
		giXStatus 	= a1.ReadStatus() ;
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	giYStatus 	= a2.ReadStatus() ;
	if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a2.Reset() ;
		sleep(1) ;
		giYStatus 	= a2.ReadStatus() ;
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	//
	// Clear the modbus memory array:
	memset(mbus_write_in.regArr,0x0,250) ;
	LOGI <<  10 << LEND;
	return;
}
/*
============================================================================
 Function:				MainClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed before the application progra, is
 terminated.
============================================================================
*/
void MainClose()
{
//
//	Here will come code for all closing processes
//
	cHost.MbusStopServer() ;
	MMC_CloseConnection(gConnHndl) ;
	return;
}
/*
============================================================================
 Function:				MachineSequences()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts the Main Timer function that will execute the states machines
 to control the system. Also performs a slow background loop for
 less time-critical background processes and monitoring of requests
 to terminate the application.
============================================================================
*/
void MachineSequences()
{
//
//	Init all variables of the states machines
//
	MachineSequencesInit();
//
//	Enable MachineSequencesTimer() every TIMER_CYCLE ms
//
	EnableMachineSequencesTimer(TIMER_CYCLE);
//
//	Background loop. Handles termination request and other less time-critical background proceses
//
	while (!giTerminate)
	{
		MachineSequencesTimer(0);
//
//		Execute background process if required
//
		BackgroundProcesses();
//
//		Sleep for ~SLEEP_TIME micro-seconds to reduce CPU load
//
		usleep(SLEEP_TIME);
	}
//
//	Termination requested. Close what needs to be cloased at the states machines
//
	MachineSequencesClose();

	return;		// Back to the main() for program termination
}
/*
============================================================================
 Function:				MachineSequencesInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the states machines variables
============================================================================
*/
void MachineSequencesInit()
{
//
//	Initializing all variables for the states machines
//
	giTerminate 	= FALSE;

	giState1 		= eIDLE;
	giPrevState1 	= eIDLE;
	giSubState1 	= eIDLE;
	//
	giState2 		= eIDLE;
	giPrevState2 	= eIDLE;
	giSubState2 	= eIDLE;

	giReentrance = FALSE;

	return;
}
/*
============================================================================
 Function:				MachineSequencesClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void MachineSequencesClose()
{
//
//	Here will come code for all closing processes
//
	return;
}
/*
============================================================================
 Function:				BackgroundProcesses()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Performs background processes that are not managed by the States Machines.
 This can be saving logs to the FLASH, performing background calculations etc.
 Generally speaking, this function, although colored in red in the Close all that needs to be closed at the states machines before the
 application program is terminated.
============================================================================
*/
void BackgroundProcesses()
{
//
//	Here will come code for all closing processes
	const auto time = std::chrono::system_clock::now();
	auto hour = std::chrono::duration_cast<std::chrono::hours>(time.time_since_epoch()).count() % 24;
	auto minute = std::chrono::duration_cast<std::chrono::minutes>(time.time_since_epoch()).count() % 60;
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() % 60;
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count() % 1000;
	std::cout <<"[time] "<< hour << ":" << minute << ":" << seconds << ":" << milliseconds << std::endl;
	return;
}
/*
============================================================================
 Function:				EnableMachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Enables the main machine sequences timer function, to be executed each
 TIMER_CYCLE ms.
============================================================================
*/
void EnableMachineSequencesTimer(int TimerCycle)
{
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
	timer.it_interval.tv_sec 	= 0;
	timer.it_interval.tv_usec 	= TimerCycle * 1000;// From ms to micro seconds
	timer.it_value.tv_sec 		= 0;
	timer.it_value.tv_usec 		= TimerCycle * 1000;// From ms to micro seconds

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
void MachineSequencesTimer(int iSig)
{
//
//	In case the application is waiting for termination, do nothing.
//	This can happen if giTerminate has been set, but the background loop
//	didn't handle it yet (it has a long sleep every loop)
//
	if (giTerminate == TRUE) return;
//
//	Avoid reentrance of this time function
//
//	Reentrance can theoretically happen if the execution of this timer function
//	is wrongly longer than TIMER_CYCLE. In such case, reentrance should be avoided
//	to prevent unexpected behavior (the code is not designed for reentrance).
//
//	In addition, some error handling should be taken by the user.
//
	if (giReentrance)
	{
//
//		Print an error message and return. Actual code should take application related error handling
//
		printf("Reentrancy!\n");

		return;
	}

	giReentrance = TRUE;		// to enable detection of reentrancy. The flag is cleared at teh end of this function
//
//	Read all input data.
//
//	Here, every TIMER_CYCLE ms, the user should read all input data that may be
//	required for the states machine code and copy them into "mirror" variables.
//
//	The states machines code, below, should use only the mirror variables, to ensure
//	that all input data is synchronized to the timer event.
//
//	Input data can be from the Host (MODBUS) or from the drives or I/Os units
//	(readingfrom the GMAS core firmware using one of the Function Blocks library
//	functions) or from any other source.
//
	ReadAllInputData();
/*
============================================================================

	States Machines code starts here!

============================================================================
*/

//
//	In case it is a new state value, clear also the value of the sub-state
//	to ensure it will start from its beginning (from the first ssub-state)
//
	if (giState1 != giPrevState1)
	{
		giSubState1 	= FIRST_SUB_STATE;
		giPrevState1 	= giState1;
	}
//
	if (giState2 != giPrevState2)
	{
		giSubState2 	= FIRST_SUB_STATE;
		giPrevState2 	= giState2;
	}
//	Handle the main state machine.
//
//	The value of the State variable is used to make decisions of the main states machine and to call,
//	as y, the relevant function that handles to process itslef in a sub-state machine.
//
	switch (giState1)
	{
//
//		Do nothing, waiting for commands
//
		case eIDLE:
		{

		if (MBUS_PACKET_FLAG == TRUE){
			// recieve packet from hostPC
			mbus_connection_counter += 1;
			mbus_timeout_counter =0;
			if (mbus_connection_counter > MBUS_CONNCETION_SUCESS_LIM){
				// If recieving is stationary
				mbus_connection_counter = MBUS_CONNCETION_SUCESS_LIM;
				cout << " _c_ " << mbus_connection_counter << " _ ";
				giState1 = eSM2;
			}
			else{
				break;
			}
		}
		else{
			cout << "Nonrecieve";
			// hostPC doesn't send signal ( or failed to recieve packet accidentary)
			mbus_timeout_counter +=1;
			if (mbus_timeout_counter > MBUS_CONNCETION_TIMEOUT_LIM){
				// Treat Connection false as " hostPC doesn't send signals "
				mbus_timeout_counter = MBUS_CONNCETION_TIMEOUT_LIM;
				mbus_connection_counter = 0;
				cout << " _out_ ";
				// // reset axis
				// control_a1.reset_integral();
				// control_a2.reset_integral();
				// control_a3.reset_integral();
				giState1 = eIDLE;
				break;
			}else{
				cout << "tmp?cnt_is_" << mbus_timeout_counter;
				// Treat Connection false as "Temporal"
				giState1 = eSM2;
			}
		}
		MBUS_PACKET_FLAG = FALSE;

			break;
		}
//
//		Do State Machine1
//
		case eSM1:
		{
			StateFunction_1();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		Do State Machine2
//
		case eSM2:
		{
			cout << "eSM2 ";
			// control_a1.p_pi_controlAxis(a1);
			// control_a2.p_pi_controlAxis(a2);
			// control_a3.p_pi_controlAxis(a3);
			StateFunction_2();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}
//
// 2nd state machine.
	switch (giState2)
	{
//
//		Do nothing, waiting for commands
//
		case eIDLE:
		{
			break;
		}
//
//		Do State Machine1
//
		case eSM1:
		{
			StateFunction_1();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		Do State Machine2
//
		case eSM2:
		{
			StateFunction_2();			// calls a sub-state machine function to handle this proocess
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}

	WriteAllOutputData();
	giReentrance = FALSE;
	return;		// End of the sequences timer function. The function will be triggered again upon the next timer event.
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
void ReadAllInputData()
{
	MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT 	mbus_read_out;
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
	cHost.MbusReadHoldingRegisterTable(MODBUS_READ_OUTPUTS_INDEX,MODBUS_READ_CNT,mbus_read_out) ;
	//
	// TODO: Extract Data read from Modbus.
	//
	giXStatus 	= a1.ReadStatus() ;
	giYStatus 	= a2.ReadStatus() ;

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
void WriteAllOutputData()
{
	//
	//	Here should come the code to write/send all ouput data
	//
	mbus_write_in.startRef 		= MODBUS_UPDATE_START_INDEX	;       // index of start write modbus register.
	mbus_write_in.refCnt 		= MODBUS_UPDATE_CNT			;		// number of indexes to write
	//
	// TODO: Prepare Data to be writen to Modbus.
	//
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in) ;
	return ;
}
/*
============================================================================
 Function:				StateFunction_1()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the Function1 process.

 For instance, a homing state machine will consist of:

	 Change Operation Mode.
	 Power Enable
	 Start homing - method number n
	 Wait for end of homing

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_1()
{
//
//	Handle the sub-state machine.
//
//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
//	as necessary, the relevant function that handles to process itself.
//
	switch (giSubState1)
	{
//
//		Perform sub SM 1
//
		case eSubState_SM1_1:
		{
			SubState1_1Function();
			break;
		}
//
//		Perform sub SM 2
//
		case eSubState_SM1_2:
		{
			SubState1_2Function();
			break;
		}
//
//		Perform sub SM 3
//
		case eSubState_SM1_3:
		{
			SubState1_3Function();
			break;
		}
//
//		Perform sub SM 4
//
		case eSubState_SM1_4:
		{
			SubState1_4Function();
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}
//
	return;
}
/*
============================================================================
 Function:				StateFunction_2()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A sub-states machine function. This function executes the sub-states machine
 of the XY move process.

 The move prcess, in this simplified example consists of the following steps:

 Begin move
 Wait for end of motion

 Each step is handled by a dedicated function. However, calling a function
 is not a must and the relevant code for each sub-state can be directly
 written within the switch-case structure.
============================================================================
*/
void StateFunction_2()
{
//
//	Handle the sub-state machine.
//
//	The value of the Sub-State variable is used to make decisions of the sub-states machine and to call,
//	as necessary, the relevant function that handles to process itslef.
//
	switch (giSubState1)
	{
//
//		Begin move
//
		case eSubState_SM2_1:
		{
			SubState2_1Function();
			break;
		}
//
//		Wait for end of motion
//
		case eSubState_SM2_2:
		{
			SubState2_2Function();
			break;
		}
//
//		The default case. Should not happen, the user can implement error handling.
//
		default:
		{
			break;
		}
	}
//
	return;
}
/*
============================================================================
 Function:				SubState1_1Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts a motion to the reverse limits of X and Y, as part of the
 states machine code for X,Y Homing, and chage the sub state to wait for the
 limits.
 ============================================================================
*/
void SubState1_1Function()
{
//
//	Here will come the code to start the relevant motions
//

//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_2;

	return;
}
/*
============================================================================
 Function:				SubState1_2Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y limits to be in motion - only then move to the next sub-state.
  ============================================================================
*/
void SubState1_2Function()
{
//
//	Changing to the next sub-state only if both axes are in motion.
//
//	Note that a faster implementation could be to put here the code of the next sub-state as well.
//	This means that if X and Y are both in motion, we can start the next state machine without
//	waiting for the next timer event.
//
//	However, this is only a Sample project to demonstrate programming guidelines and concepts,
//	so we preferred to keep it as simple as possible.
//
	if (giXInMotion && giYInMotion)
	{
		giSubState1 = eSubState_SM1_3;
	}

	return;
}
/*
============================================================================
 Function:				SubState1_3Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts a motion to the index of X and Y, as part of the
 states machine code for X,Y Homing, and chage the sub state to wait for the
 indexes.
 ============================================================================
*/
void SubState1_3Function()
{
//
//	Here will come the code to start the relevant motions
//

//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_4;

	return;
}
/*
============================================================================
 Function:				SubState1_4Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y motions to end.
  ============================================================================
*/
void SubState1_4Function()
{
//
//	Ending state machine only if both axes are not in motion.
//
	if (!giXInMotion && !giYInMotion)
	{
		giState1 = eIDLE;
	}

	return;
}
/*
============================================================================
 Function:				SubState2_1Function()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Starts an XY motion as part of the states machine code for X,Y move,
 and chage the sub state to wait for the end of motions.
 ============================================================================
*/
void SubState2_1Function()
{
//
//	Here will come the code to start the relevant motions
//

//
//	Changing to the next sub-state
//
	giSubState1 = eSubState_SM1_2;

	return;
}
/*
============================================================================
 Function:				SubStateXYMoveWaitEndMotionFunction()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Waits for X and Y to stop motions and only then finishes the XY move process.
  ============================================================================
*/
void SubState2_2Function()
{
//
//	Ending X,Y move only if both indexes are activated.
//
	if ( (~giXInMotion) && (~giYInMotion) )
	{
		giState1 = eIDLE;
	}

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
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	std::cout << "callback called" << std::endl;
	// Which function ID was received ...
	switch(recvBuffer[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n") ;
		break ;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n") ;
		break ;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n") ;
		break ;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n") ;
		break ;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n") ;
		break ;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n") ;
		break ;
	/* This is commented as a specific event was written for this function. Once it occurs
	 * the ModbusWrite_Received will be called
		case MODBUS_WRITE_EVT:
		// TODO Update additional data to be read such as function parameters.
		// TODO Remove return 0 if you want to handle as part of callback.
		return 0;
		printf("Modbus Write Event received - Updating Outputs\r\n") ;

		break ;
	*/
	default:
		std::cout <<" hoghoge" << std::endl;
	}
	//
	return 1 ;
}

int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
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
void TerminateApplication(int iSigNum)
{
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1 ;
	sigignore(SIGALRM);
	//
	// Wait till other threads exit properly.
	sleep(1) ;
	return ;
}

//
// Callback Function once a Modbus message is received.
void ModbusWrite_Received()
{
	printf("Modbus Write Received\n") ;
}
//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}
