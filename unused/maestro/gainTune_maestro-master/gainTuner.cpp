#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "gainTuner.h"		// Application header file.
#include "TorControls.h"
#include <iostream>
#include <sys/time.h>			// For time structure
#include <stdio.h>
#include <ctime>			// modbus time
#include <signal.h>				// For Timer mechanism

using namespace std;

// For timestamp
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <termios.h>

// Add for tunner
#include <stdlib.h>


int main(int argc, char *argv[])
{
	a_id = atoi(argv[1]) -1;	// id
	if (a_id == 0){
		cout << "Axis ID is 1, 2, 3, 4, ... 10" << endl;
	}
	if (argc != 2){
		cout << "Set Axis ID correctory" << endl;
	}
	MainInit(); //	Initialize system, axes and all needed initializations
	powerOnAxis(a1);
	cout << "Power on "<< a_candidate[a_id] <<endl;
	MachineSequences(); //	Execute the state machine to handle the system sequences and control
	powerOffAxis(a1);
	MainClose(); //	Close what needs to be closed before program termination
	return 0;
}


void MainInit() // Initilaize the system, including axes, communication, etc.
{
	// make connection
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	struct sigaction stSigAction;
	memset(&stSigAction, 0, sizeof(stSigAction));	// Init the sigAction structure.
	stSigAction.sa_handler = &TerminateApplication;
	sigaction(SIGINT, &stSigAction, NULL); 	// Initialized case of CTRL+C.

	// Initialize modbus
	cHost.MbusStartServer(gConnHndl,1) ;// Start the Modbus Server:
	cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE,(void*)ModbusWrite_Received) ; // callback-1
	cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ; // callback-2
	memset(mbus_write_in.regArr,0x0,250) ; //Clear the modbus memory array:
	MBUS_PACKET_FLAG = FALSE;

	// Others
	CMMCPPGlobal::Instance()->SetThrowFlag(true,false); // 	Enable throw feature. @ axis
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);// Register Run Time Error Callback function @ modbus

	// TO DO ... define axis
	a1.InitAxisData(a_candidate[a_id],gConnHndl) ;
	initAxisRoutin(a1);

	cout << "End initialization " << endl;
	return;
}


void initAxisRoutin(CMMCSingleAxis axs){
	// Default Params
	int Status 	= axs.ReadStatus() ;
	if(Status & NC_AXIS_ERROR_STOP_MASK)
	{
		axs.Reset() ;
		Status 	= axs.ReadStatus() ;
		if(Status & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis is in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	return;
}

void powerOnAxis(CMMCSingleAxis axs){
	// mode change
	int ret = axs.SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
	cout << "change is successed if ret == 0, and ret is ..." << ret << endl;
	if (ret != 0){
		cerr << "Set error" << endl;
	}else{
		cout << "Mode is 10 ? ... " << axs.GetOpMode() << endl;
	}
	// Axis idling state
	axs.PowerOn();
	int Status 	= axs.ReadStatus() ;
	if(Status & NC_AXIS_ERROR_STOP_MASK)
	{
		axs.Reset() ;
		Status 	= axs.ReadStatus() ;
		if(Status & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis is in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	// Cyclic torque control mode
	cout << "power on is completed" << endl;
	return;
}



void powerOffAxis(CMMCSingleAxis axs){
	axs.PowerOff();
	int Status 	= axs.ReadStatus() ;
	if(Status & NC_AXIS_ERROR_STOP_MASK)
	{
		axs.Reset() ;
		Status 	= axs.ReadStatus() ;
		if(Status & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis is in Error Stop. Aborting." ;
			exit(0) ;
		}
	cout << " OFF";
	}
	return;
}

void MachineSequences()
{
	MachineSequencesInit();//	Init all variables of the states machines
	cout << "Done machine seq init" << endl;
	EnableMachineSequencesTimer(TIMER_CYCLE); //	Enable MachineSequencesTimer() every TIMER_CYCLE ms
	cout << "Start while loop" << endl;
	while (!giTerminate) //	Background loop. Handles termination request and other less time-critical background proceses
	{
		MachineSequencesTimer(0);
		BackgroundProcesses(); //		Execute background process if required
		usleep(SLEEP_TIME); //		Sleep for ~SLEEP_TIME micro-seconds to reduce CPU load
	}
	MachineSequencesClose(); //	Termination requested. Close what needs to be cloased at the states machines
	return;		// Back to the main() for program termination
}

void MainClose()
{
	cHost.MbusStopServer() ;			// kill mbus server
	MMC_CloseConnection(gConnHndl) ;	// kill gmas process
	return;
}


// IN MachineSequences()
void MachineSequencesInit() //	Initializing all variables for the states machines
{
	// Set Initial state state
	giTerminate 	= FALSE;
	giState1 		= eIDLE;
	giPrevState1 	= eIDLE;
	giReentrance = FALSE;
	mbus_connection_counter = 0;
	mbus_timeout_counter = MBUS_CONNCETION_TIMEOUT_LIM;
	fromTorqueControlMode = FALSE;

	return;
}

void MachineSequencesTimer(int iSig)
{
	if (giTerminate == TRUE) return; //	Avoid reentrance of this time function
	if (giReentrance){
		printf("Reentrancy!\n");
		return;}//	Error handling should be taken by the user.
	giReentrance = TRUE;		// to enable detection of reentrancy.
	ReadAllInputData(); //	Read all input data.

	if (giState1 != giPrevState1)
	{
		giPrevState1 	= giState1;
	}

	switch (giState1)
	{
		case eIDLE:	//Recieve the packet of modbus
		{
			if (MBUS_PACKET_FLAG == TRUE){
				// recieve packet from hostPC
				mbus_connection_counter += 1;
				mbus_timeout_counter =0;
				if (mbus_connection_counter > MBUS_CONNCETION_SUCESS_LIM){
					// If recieving is stationary
					mbus_connection_counter = MBUS_CONNCETION_SUCESS_LIM;
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
					// reset axis
					control_a1.reset_integral();
					giState1 = eIDLE;
					break;
				}else{
					cout << "tmp?cnt_is_" << mbus_timeout_counter;
					// Treat Connection false as "Temporal"
					giState1 = eSM2;
				}
			}
			MBUS_PACKET_FLAG = FALSE;		// for Re-entrance avoidance
		}
		case eSM2:			//Do Contorl Axis
		{
			control_a1.p_pi_controlAxis(a1);
			giState1 = eIDLE;
			break;
		}
		default:			//The default case. Should not happen, the user can implement error handling.
		{
			cout << "<<default>>";
			giState1 = eIDLE;
			break;
		}
	}
	WriteAllOutputData();
	giReentrance = FALSE;//	Clear the reentrancy flag
	return;		// End of the sequences timer function.
}


void ReadAllInputData()
{
	cHost.MbusReadHoldingRegisterTable(MODBUS_READ_OUTPUT_INDEX, MODBUS_READ_CNT, mbus_read_out);
	control_a1.set_CommandFromHost(mbus_read_out.regArr, eCommand1);
	control_a1.set_GainAgan(mbus_read_out.regArr, eKP_pos);

	if (mbus_connection_counter == 5){
		control_a1.set_GainAgan(mbus_read_out.regArr, eKP_pos);
	}
	return;
}

int read_32bit_from_mbus16bit(short *reg, int startRef){
	int hi = (reg[startRef]<<16)>>16;
	int lo = (reg[startRef+1]<<16)>>16;
	int ret = hi*32767 + lo;
	return ret;
}

void WriteAllOutputData()
{
	mbus_write_in.startRef 		= MODBUS_WRITE_IN_INDEX	;       	// index of start write modbus register.
	mbus_write_in.refCnt 		= MODBUS_WRITE_IN_CNT			;			// number of indexes to write
	write_time_to_mbus(CLOCK_MONOTONIC_COARSE, eTimeStartId);// write timestamp
	write_axis_info_to_mbus(a1, eAx1);
	write_TorControlVals_to_mbus(control_a1, eVelOrder);
	cHost.MbusWriteHoldingRegisterTable(mbus_write_in) ;
	return ;
}

void write_time_to_mbus(clockid_t clk_id, int start_ref)
{
    struct timespec tvToday; // for msec
    struct tm *ptm; // for date and time
    clock_gettime(clk_id, &tvToday);
    ptm = localtime(&tvToday.tv_sec);
    h_ = double(ptm->tm_hour);
    m_ = double(ptm->tm_min);
    s_ = double(ptm->tm_sec);
    ms_ = double(tvToday.tv_nsec / 1000000);
    send_64bit_to_mbus16bit(h_, mbus_write_in.regArr, start_ref);
    send_64bit_to_mbus16bit(m_, mbus_write_in.regArr, start_ref+2);
    send_64bit_to_mbus16bit(s_, mbus_write_in.regArr, start_ref+4);
    send_64bit_to_mbus16bit(ms_, mbus_write_in.regArr, start_ref+6);
    return;
}

void write_TorControlVals_to_mbus(TorControls controller, int start_ref){
	double tmp_Vorder = controller.get_v_order();
	double tmp_TorOrder =controller.get_tor_order();
    send_64bit_to_mbus16bit(tmp_Vorder, mbus_write_in.regArr, start_ref);
    send_64bit_to_mbus16bit(tmp_TorOrder, mbus_write_in.regArr, start_ref+2);
    return;
}

void write_axis_info_to_mbus(CMMCSingleAxis arg_axis, int start_ref){
	double tmp_pos = arg_axis.GetActualPosition();
	double tmp_vel = arg_axis.GetActualVelocity();
	double tmp_tor = arg_axis.GetActualTorque() * 1000.; // mA
    send_64bit_to_mbus16bit(tmp_pos, mbus_write_in.regArr, start_ref);
    send_64bit_to_mbus16bit(tmp_vel, mbus_write_in.regArr, start_ref+2);
    send_64bit_to_mbus16bit(tmp_tor, mbus_write_in.regArr, start_ref+4);
	return;
}
/////////////////////////////////////////////////////////////
// //////////////////////Utils//////////////////////////////
/////////////////////////////////////////////////////////////
int read_64bit_from_mbus16bit(short *reg, int startRef){
	int hi = (reg[startRef]<<16)>>16;
	int lo = (reg[startRef+1]<<16)>>16;
	int ret = hi*32767 + lo;
	// int ret = reg[startRef]*32767 + reg[startRef+1];
	return ret;
}
void send_64bit_to_mbus16bit(double arg_value, short* regArr_of_mbus, int startRef){
	// Convert double(64 bit) to int(32 bit) .... LREAL to DINT
	int int_tmp_int = int(arg_value);
	short v1 = short(int_tmp_int/32767); // 32767 = pow(2, 15) -1
	short v2 = short(int_tmp_int - int(v1)*32767);
	regArr_of_mbus[startRef] = v1;
	regArr_of_mbus[startRef+1] = v2;
};



///////////////////////////////////////////////////////////////////////////////////////////////
// OMAJINAI ///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

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
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}
void ModbusWrite_Received()
{
//	cout << "R";
	MBUS_PACKET_FLAG = TRUE;
//	printf("Modbus Write Received : ") ;
}

void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}


void MachineSequencesClose()//	Here will come code for all closing processes
{
	cout << "End process" << endl;
	return;
}

void BackgroundProcesses()//	Here will come code for all closing processes
{
	cout << " Time " << h_ << ":" << m_ << ":" << s_ << ":" << ms_ <<endl;
	return;
}

void EnableMachineSequencesTimer(int TimerCycle)
{
	struct itimerval timer;
	struct sigaction stSigAction;

	stSigAction.sa_handler = TerminateApplication; // Whenever a signal is caught, call TerminateApplication function
	sigaction(SIGINT, &stSigAction, NULL);
	sigaction(SIGTERM, &stSigAction, NULL);
	sigaction(SIGABRT, &stSigAction, NULL);
	sigaction(SIGQUIT, &stSigAction, NULL);

	//	Enable the main machine sequences timer function
	timer.it_interval.tv_sec 	= 0;
	timer.it_interval.tv_usec 	= TimerCycle * 1000;// From ms to micro seconds
	timer.it_value.tv_sec 		= 0;
	timer.it_value.tv_usec 		= TimerCycle * 1000;// From ms to micro seconds
	cout << "EnableMachineSequencesTimer" << endl;
	return;
}

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
