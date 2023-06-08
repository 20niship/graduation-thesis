# include <math.h>
# include "TorControls.h"

// FUNCTION HIERARCY (In main)
void MainInit();
	int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
	int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
	void ModbusWrite_Received();
	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);
	void initAxisRoutin(CMMCSingleAxis axs); // user define
void powerOnAxis(CMMCSingleAxis axs);
void MachineSequences();
	void MachineSequencesInit();
	void EnableMachineSequencesTimer(int TimerCycle);
		void TerminateApplication(int iSigNum); // Prepare for Successful completion
	// WHILE LOOP STOP : giTerminate != FALSE
		void MachineSequencesTimer(int TimerCycle);
			void ReadAllInputData(); // Read & Decode data sent from modbus
			// Torque Control process defined by "TorControls.h"
			void WriteAllOutputData(); // Encode & Write data to modbus
				void write_time_to_mbus(clockid_t clk_id, int start_ref); // Set Timestamp
				void write_axis_info_to_mbus(CMMCSingleAxis arg_axis, int start_ref); // Set axis data
		void BackgroundProcesses();
	// END WHILE LOOP
	void MachineSequencesClose();
void powerOffAxis(CMMCSingleAxis axs);
void MainClose();

// Modbus interface
void send_64bit_to_mbus16bit(double arg_value, short* regArr_of_mbus, int startRef);
int read_32bit_from_mbus16bit(short *reg, int startRef);
// kbhit functions
void init_keyboard();
void close_keyboard();
int kbhit();
int readkey();
int ch;		// for keyboard hit detector

// Application global variables
int 	giTerminate;		// Flag to request program termination
int		giReentrance;		// Used to detect reentrancy to the main timer function
// State
int 	giState1;			// Holds the current state of the 1st main state machine
int 	giNextStartState;
int 	giPrevState1;		// Holds the value of giState1 at previous cycle
int		giSubState1;		// Holds teh current state of the sub-state machine of 1st main state machine


// MODBUS CONFIG
#define MODBUS_WRITE_IN_INDEX 0			// write in is regArr[MODBUS_WRITE_IN_INDEX]
#define MODBUS_WRITE_IN_CNT 100			// to regArr[MODBUS_WRITE_IN_INDEX + MODBUS_WRITE_IN_CNT -1]
#define MODBUS_READ_OUTPUT_INDEX 0	// read_out is regArr[MODBUS_READ_OUTPUT_INDEX]
#define MODBUS_READ_CNT 100				// to regArr[MODBUS_READ_OUTPUT_INDEX + MODBUS_READ_CNT -1]

// MODBUS ARRAY ID
//#define MODBUS_TIME_START_INDEX 0 // 0 <= id <8 is h, m, s, sec

enum mbusStartIdlist
{
	eTimeStartId			= 	0,						// regArr[eTimeStart] ~ regArr[eTimeStart+8] is Time
	eAx1					= 	10,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
	eAx2					= 	16,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
	eAx3					= 	22,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
//	eAx4					= 	28,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
//	eAx5					= 	34,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
//	eAx6					= 	40,						// regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
	eCommand1 				= 	50,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,
	eCommand2 				= 	54,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,
	eCommand3 				= 	58,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,
//	eCommand4 				= 	62,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,
//	eCommand5 				= 	66,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,
//	eCommand6 				= 	70,						// regArr[eCommand1] ~ regArr[eCommand1] is target_pos, torlim,

} ;

// Valiables
//int localRegister[MODBUS_IPC_WRITE_VALUES]; // Define 250 valiables to manage local register
double h_;
double m_;
double s_;
double ms_;

// Time setup
//#define		SLEEP_TIME				9000	// micro seconds. default is 10000 == 10 ms
#define		SLEEP_TIME				10000	// micro seconds. default is 10000 == 10 ms
#define		TIMER_CYCLE				1		// Cycle time of the main sequences. In ms. default is 20

// Bool Index (User defined)
#define 	FALSE					0
#define 	TRUE					1

// State Index (int type)
#define 	FIRST_SUB_STATE			1
int			MBUS_PACKET_FLAG;
#define		MBUS_CONNCETION_SUCESS_LIM 10	//about 0.1sec
int			mbus_connection_counter;
#define		MBUS_CONNCETION_TIMEOUT_LIM 10	//about 0.1sec
int			mbus_timeout_counter;
int 		fromTorqueControlMode;
//int			MBUS_RECIEVE_PACKET_SUM;

enum eMainStateMachines						// TODO: Change names of state machines to reflect dedicated project
{
	eIDLE		= 	0,						// Dealing with modbus signals from hostPC
	eSM1		=	1,						// For the termination of host-client connection
	eSM2		= 	2,						// For the torque control state
} ;

MMC_CONNECT_HNDL gConnHndl ;					// Connection Handle
CMMCConnection cConn ;
CMMCSingleAxis a1, a2, a3, a4, a5, a6 ;							// TODO : Update the names and number of the axes in the system
CMMCHostComm	cHost ;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN 	mbus_write_in;// Having regArr
MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT 	mbus_read_out; //Having regArr
MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data

// Set Torque Control  .... KP[1], KP[2], KI[2]
TorControls control_a1(166, 0.095*pow(10,-5),	1.5,	2000);		// default is 240, 1.85*pow(10,-5), 29.7 1/10 is good?
TorControls control_a2(176, 0.42*pow(10,-5),	2.07,	5000);		// default is 160, 3.91*pow(10,-5), 18.5
TorControls control_a3(129, 0.31*pow(10,-5),	1.6,	5000);		// default is 160, 3.91*pow(10,-5), 18.5

// For torque control
OPM402    drvMode;		// Default is 8 == OPM402_CYCLIC_SYNC_POSITION_MODE
short     sErrorID;

