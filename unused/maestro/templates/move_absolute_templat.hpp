/*
============================================================================
  Name : move_absoulte_template.cpp
 Author :	Elmo Motion Control
 Version :	1.00
 Description : The following example supports the following functionalities:

- Power On.
- Move Absolute to 10000.
- Move Absolute to 0.
- Power off.

 The program works with 2 axes - a01 and a02.
 The program works with CAN and EtherCAT.
============================================================================
*/
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "move_absoulte_template.h"		// Application header file.
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				04/10/2015
 Modifications:			N/A

 Description:

 The main function of this sample project.
============================================================================
*/

int main()
{
	try
	{
		//	Initialize system, axes and all needed initializations
		MainInit();
		//
		/////////////////////////////////////// Motor On function - axis 1//////////////////////////////////////////
		//
		a1.PowerOn();
		//
		// Waiting for Motor on.
		// In the end of Motor On function the axis should be in StandStill state.
		giXStatus = a1.ReadStatus();
		while ( !(giXStatus & NC_AXIS_STAND_STILL_MASK) && !(giXStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giXStatus = a1.ReadStatus();
		//
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		cout<< "Axis a1 is powered on" <<endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		////////////////////////////////////////// Axis 1 - Move to 10000 ////////////////////////////////////////////
		//
		a1.MoveAbsolute(10000);
		//
		// Waiting for Move Absolute.
		// In the end of Move Absolute function the axis should be in StandStill state.
		giXStatus = a1.ReadStatus();
		while ( !(giXStatus & NC_AXIS_STAND_STILL_MASK) && !(giXStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giXStatus = a1.ReadStatus();
		//
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		////////////////////////////////////////// Axis 1 - Move to 0 ///////////////////////////////////////////////
		//
		a1.MoveAbsolute(0);
		//
		// Waiting for Move Absolute.
		// In the end of Move Absolute function the axis should be in StandStill state.
		giXStatus = a1.ReadStatus();
		while ( !(giXStatus & NC_AXIS_STAND_STILL_MASK) && !(giXStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giXStatus = a1.ReadStatus();
		//
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		/////////////////////////////////////// Motor On function - axis 2///////////////////////////////////////////
		//
		a2.PowerOn();
		//
		// Waiting for Motor on.
		// In the end of Motor On function the axis should be in StandStill state.
		giYStatus = a2.ReadStatus();
		while ( !(giYStatus & NC_AXIS_STAND_STILL_MASK) && !(giYStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giYStatus = a2.ReadStatus();
		//
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		cout<< "Axis a2 is powered on" <<endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		////////////////////////////////////////// Axis 2 - Move to 10000////////////////////////////////////////////
		//
		// Move to 10000.
		a2.MoveAbsolute(10000);
		//
		// Waiting for Move Absolute.
		// In the end of Move Absolute function the axis should be in StandStill state.
		giYStatus = a2.ReadStatus();
		while ( !(giYStatus & NC_AXIS_STAND_STILL_MASK) && !(giYStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giYStatus = a2.ReadStatus();
		//
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		////////////////////////////////////////// Axis 2 - Move to 10000////////////////////////////////////////////
		//
		// Move back to 0.
		a2.MoveAbsolute(0);
		//
		// Waiting for Move Absolute.
		// In the end of Move Absolute function the axis should be in StandStill state.
		giYStatus = a2.ReadStatus();
		while ( !(giYStatus & NC_AXIS_STAND_STILL_MASK) && !(giYStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giYStatus = a2.ReadStatus();
		//
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		/////////////////////////////////////// Motor Off function - axis 1//////////////////////////////////////////
		//
		a1.PowerOff();
		//
		// Waiting for Motor off.
		// In the end of Motor Off function the axis should be in Disabled state.
		giXStatus = a1.ReadStatus();
		while ( !(giXStatus & NC_AXIS_DISABLED_MASK) && !(giXStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giXStatus = a1.ReadStatus();
		//
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		cout<< "Axis a1 is powered off" <<endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		/////////////////////////////////////// Motor Off function - axis 2//////////////////////////////////////////
		//
		a2.PowerOff();
		//
		// Waiting for Motor off.
		// In the end of Motor Off function the axis should be in Disabled state.
		giYStatus = a2.ReadStatus();
		while ( !(giYStatus & NC_AXIS_DISABLED_MASK) && !(giYStatus & NC_AXIS_ERROR_STOP_MASK ) )
				giYStatus = a2.ReadStatus();
		//
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}
		//
		cout<< "Axis a2 is powered off" <<endl;
		//
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//
		// Close what needs to be closed before program termination
		MainClose();
		//
		// Terminate the application program back to the Operating System
		return 1;
	}
	catch(CMMCException& exception)
	{
		printf("Exception in function %s, axis ref=%s, err=%d, status=%d, bye\n", exception.what(), exception.axisName(), exception.error(), exception.status());
		MainClose();
		exit(0);
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught\n";
		MainClose();
		exit(0);
	}
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
	// Here will come initialization code for all system, axes, communication etc.
	//
	struct sigaction stSigAction;
	//
	// Init the sigAction structure.
	memset(&stSigAction, 0, sizeof(stSigAction));
	stSigAction.sa_handler = &TerminateApplication;
	// Initialized case of CTRL+C.
	sigaction(SIGINT, &stSigAction, NULL);
	//
	// InitializeCommunication to the GMAS:
	//
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	//
	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);
	//
	// Register the callback function for Emergency:
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
	// TODO: Update number of necessary axes:
	//
	a1.InitAxisData("a01",gConnHndl) ;
	a2.InitAxisData("a02",gConnHndl) ;
	//
	// Set default motion parameters. TODO: Update for all axes.
	a1.SetDefaultParams(stSingleDefault) ;
	a2.SetDefaultParams(stSingleDefault) ;
	//
	// You may of course change internal parameters manually:
	a1.m_fAcceleration=10000;
	//
	giXStatus 	= a1.ReadStatus() ;
	if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a1.Reset() ;
		giXStatus 	= a1.ReadStatus() ;
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a1 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	//
	giYStatus 	= a2.ReadStatus() ;
	if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a2.Reset() ;
		giYStatus 	= a2.ReadStatus() ;
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a2 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	//
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

	//	Here will come code for all closing processes

	MMC_CloseConnection(gConnHndl) ;
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
	}
	//
	return 1 ;
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case a Run Time Error was received.
//	Input			:	const char - *msg, unsigned int - uiConnHndl, unsigned short - usAxisRef, short - sErrorID, unsigned short - usStatus.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
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
	switch(iSigNum)
	{
		// Handle ctrl+c.
		case SIGINT:
			// TODO Close what needs to be closed before program termination.
			exit(0);
			break;
		default:
			break;
	}
	return ;
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case an Emergency event was received.
//	Input			:	unsigned short - usAxisRef, short - sEmcyCode.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}
