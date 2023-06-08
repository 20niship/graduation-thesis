/*
 * TorControls.cpp
 *
 *  Created on: 2019/08/08
 *      Author: takayuki-kanai
 */

#include "TorControls.h"
#include <math.h>

TorControls::TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard) {
	// TODO Auto-generated constructor stub
	KP_position = kp_pos / (2.0*M_PI);	// [rad/s] to [/s]
	KP_velocity = kp_vel * 1000.;		// [A/(cnt/s)] to [mA/(cnt/s)]
	KI_velocity = ki_vel / 1000.;		// [Hz] = [1/s] to [1/ms]
	CurLimHardware = curLimHard;
	// initialize private variables
	torControlInitializer = true;
	tor_order_integral = 0;
	torLimFlag = false;
}

TorControls::~TorControls() {
	// TODO Auto-generated destructor stub
	torControlInitializer = false;
	tor_order_integral = 0.;
}

void TorControls::set_GainAgan(short *reg, int gainStartId_mbus, bool times10000){
	// Define id
	int kp_pos_MbusId = gainStartId_mbus;
	int kp_vel_MbusId = gainStartId_mbus + 2;
	int ki_vel_MbusId = gainStartId_mbus + 4;

	// Read from mbus
	double tmp_kp_pos = read_32bit_from_mbus16bit(reg, kp_pos_MbusId)*1.0;
	double tmp_kp_vel = read_32bit_from_mbus16bit(reg, kp_vel_MbusId)*1.0;
	double tmp_ki_vel = read_32bit_from_mbus16bit(reg, ki_vel_MbusId)*1.0;

	// if multiplied 10000
	if (times10000) {
		tmp_kp_pos = 1.0 * tmp_kp_pos/100.;
		tmp_kp_vel = 1.0 * tmp_kp_vel/10000000.;
		tmp_ki_vel = 1.0 * tmp_ki_vel/100.;
	}
	KP_position = 1.0 * tmp_kp_pos / (2.0*M_PI);	// [rad/s] to [/s]
	KP_velocity = 1.0 * tmp_kp_vel * 1000.;		// [A/(cnt/s)] to [mA/(cnt/s)]
	KI_velocity = 1.0 * tmp_kp_vel / 1000.;		// [Hz] = [1/s] to [1/ms]

	//update reset flag
	reset_gainFlag = true;
}

void TorControls::set_CommandFromHost(short *reg, int torlimMbusId){
	int posCmdMbusId = torlimMbusId + 2;
	torLim_mA = read_32bit_from_mbus16bit(reg, torlimMbusId);
	target_pos = read_32bit_from_mbus16bit(reg, posCmdMbusId);
	if (torLim_mA == 0 && target_pos == 0){
		// It means modbus connection is failed tamporary
		torLim_mA = torLim_mA_old;
		target_pos = target_pos_old;
	}

	return;
}


int TorControls::read_32bit_from_mbus16bit(short *reg, int startRef){
	int hi = (reg[startRef]<<16)>>16;
	int lo = (reg[startRef+1]<<16)>>16;
	int ret = hi*32767 + lo;
	return ret;
}

void TorControls::reset_integral(){
	cout<< "RES";
	tor_order_integral = 0;
	tor_order = 0;
	torLimFlag = false;
}

void TorControls::p_pi_controlAxis(CMMCSingleAxis single_axis){
	now_pos = single_axis.GetActualPosition();
	now_vel = single_axis.GetActualVelocity();
	pos_err = (target_pos*1.0 - now_pos);
	int lim_mA = get_currentLim();

	cout << "err" << pos_err ;

	// For Abnormal End handling
	if (torControlInitializer){
		torControlState = eSetVelocityOrder;
	}else{
		torControlState = 0;
	}

	switch (torControlState)
	{
		case eSetVelocityOrder: // Set Velocity Order ... if TorTh1 >0
		{
			v_order = KP_position * pos_err;
			torControlState = eSetTorqueOrder; 		//To next state
		}
		case eSetTorqueOrder:	//Set Torque Order
		{
			tor_order = KP_velocity * (v_order - now_vel + tor_order_integral );
			if ( torLimFlag == false){
				tor_order_integral += KI_velocity * (v_order - now_vel);
			}
			torControlState = eCheckLimit; 		//To next state
		}
		case eCheckLimit:		//Set control mode ... position control mode
		{
			cout << "tor_order_1" << tor_order;
			if ( abs(tor_order) > lim_mA){
				tor_order = lim_mA * sign(tor_order) * 1.0;
				torLimFlag = true;
			}
			else{
				torLimFlag = false;
			}
			torControlState = eMoveTorque; 		//To next state
		}

		case eMoveTorque:		//Set control mode ... position control mode
		{
			single_axis.MoveTorque(tor_order, 5.0*pow(10, 6), 1.0*pow(10, 8), MC_ABORTING_MODE);
//			single_axis.MoveAbsoluteRepetitive(target_pos,MC_ABORTING_MODE);
//			cout <<"vOrd" << v_order <<  "tOrd" << tor_order << " ** ";
			// For Abnormal termination
			int axisStatus = single_axis.ReadStatus();
			if(axisStatus & NC_AXIS_ERROR_STOP_MASK)
			{
				single_axis.Reset() ;
				axisStatus 	= single_axis.ReadStatus() ;
				if(axisStatus & NC_AXIS_ERROR_STOP_MASK)
				{
					cout << "Axis a1 in Error Stop. Aborting." ;
					exit(0) ;
				}
			}
			torControlState = eErrorCheckAndReset; 		//To next state
		}

		case eErrorCheckAndReset:
		{
			torLim_mA_old = torLim_mA;
			target_pos_old = target_pos;
			pos_err = 0;
			break;

		}

		default:			//The default case. Should not happen, the user can implement error handling.
		{
			int axisStatus = single_axis.ReadStatus();
			if(axisStatus & NC_AXIS_ERROR_STOP_MASK)
						{
							single_axis.Reset() ;
							axisStatus 	= single_axis.ReadStatus() ;
							if(axisStatus & NC_AXIS_ERROR_STOP_MASK)
							{
								cout << "Axis a1 in Error Stop. Aborting." ;
								exit(0) ;
							}
						}
			torControlState = 0; 		//To next state
			break;
		}
	}
	return;
}

int TorControls::get_currentLim(){
	if (torLim_mA > CurLimHardware){
		return CurLimHardware;
	}
	else if (torLim_mA < 0){
//		cout << " minus?" << torLim_mA << " ";
		return 0;
	}
	else{
		return torLim_mA;
	}
}

double TorControls::sign(double A){
    if(A>0) return 1;
    else if(A<0) return -1;
    else return 0;
}

double TorControls::get_KP_pos(){
	return KP_position;
}

double TorControls::get_KP_vel(){
	return KP_velocity;
}

double TorControls::get_v_order(){
	return v_order;
}

double TorControls::get_tor_order(){
	return tor_order;
}

