//
// Created by takayuki on 19/08/08.
//

#ifndef TEST_AXISINTERFACE_H
#define TEST_AXISINTERFACE_H

#include <modbus/modbus.h>
#include <iostream>

#define MAESTRO_IP "192.168.2.92"
#define MODBUS_ARRAY_NUM 40     // The max number of the transportable 32bit variables by modbus is MODBUS_ARRAY_NUM/2

using namespace std;

class AxisInterface {
public:
    AxisInterface(int mbusId, int gNumerator, int gDenominator, int mbusSendCmdId = -1, int mbusTimeId = 0);

    ~AxisInterface();

    bool connection_is_ok;

    void init_encSlaveEncoder(double edata, int direction_mode);

    void send_cmd(float rad_edata, int Torlim);

    void update_sensor(bool option_mode = true, int int_mbus_id_optionVal = 26);

    int get_pos();

    int get_vel();

    int get_cur();

    int get_PosCmd();

    void get_timestamp(int int_4elem_array[]);

    void send_ElmoGains_times10000(double kp_pos, double kp_vel, double ki_vel, int mbus_param_start_id = 16);

    double get_motorOtptAxis_rad();

    int get_VelOrder();

    int get_TorOrder();

private:
    modbus_t *mb;
    uint16_t tab_reg[MODBUS_ARRAY_NUM];
    uint16_t cmd_reg[MODBUS_ARRAY_NUM];
    uint16_t gain_reg[MODBUS_ARRAY_NUM];

    int direction_mode_plus_or_minus;
    int mbus_timestamp[4];
    bool enc0_slave_rad_is_loaded;
    // Read from modbus
    int pos;
    int vel;
    int cur;
    // option variable
    int VelOrder;
    int TorOrder;
    int pos_straged_id_mbus;
    int time_straged_id;
    // Necessary for send order to elmo driver
    double gRatio;
    int pos0_motorInpt;
    double enc0_slave_rad;
    int PosCmd;
    int Torlim_straged_id_mbus;


    // user utils
    void set_ElmoCmd(int TorCmd, int PosCmd, uint16_t *reg);

    int read_32bit_from_mbus16bit(uint16_t *reg, int startRef);

    void send_32bit_to_mbus16bit(double arg_value, uint16_t *regArr_of_mbus, int startRef);


};


#endif //TEST_AXISINTERFACE_H
