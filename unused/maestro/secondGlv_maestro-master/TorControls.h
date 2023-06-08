/*
 * TorControls.h
 *
 *  Created on: 2019/08/08
 *      Author: takayuki-kanai
 */

#ifndef TORCONTROLS_H_
#define TORCONTROLS_H_

#include "mmc_definitions.h"
#include "mmcpplib.h"

class TorControls {
public:
	TorControls(double kp_pos, double kp_vel, double ki_vel, double curLimHard=1000);
	virtual ~TorControls();
	void set_CommandFromHost(short *reg, int torlimMbusId);
	void p_pi_controlAxis(CMMCSingleAxis single_axis);
	void reset_integral();
	double get_v_order();
	double get_tor_order();

private:
	bool torControlInitializer;
	double KP_position;			// [/s]
	double KP_velocity;			// [mA/(cnt/s)]
	double KI_velocity;			// [/ms]
	double CurLimHardware;		// [mA]
	int target_pos;
	int torLim_mA;
	int target_pos_old;
	int torLim_mA_old;
	//For temporals to use torControl
	double now_pos, now_vel;
	double pos_err, v_order,tor_order, tor_order_integral;
	bool torLimFlag;
	int torControlState;
	// User utils
	double sign(double A);
	int read_32bit_from_mbus16bit(short *reg, int startRef);
	int get_currentLim();
};

enum eStateTorqueControl	// TODO: Change names of sub-state machines.
{
	eSetVelocityOrder 	=	1,
	eSetTorqueOrder 	=	2,
	eCheckLimit			=	3,
	eMoveTorque		 	=	4,
	eErrorCheckAndReset	=	5,
};

#endif /* TORCONTROLS_H_ */
