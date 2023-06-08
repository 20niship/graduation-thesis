//
// Created by takayuki on 19/08/08.
//

#include "AxisInterface.h"
#include <modbus/modbus.h>
#include <math.h>

AxisInterface::AxisInterface(int mbusRefId, int gNumerator, int gDenominator, int mbusSendCmdId,
                             int mbusTimeId) {
    pos_straged_id_mbus = mbusRefId;
    if (mbusSendCmdId == -1) {
        Torlim_straged_id_mbus = pos_straged_id_mbus + 6;

    } else {
        Torlim_straged_id_mbus = mbusSendCmdId;
    }
    time_straged_id = mbusTimeId;

    for (int i = 0; i < MODBUS_ARRAY_NUM; i++) {
        tab_reg[MODBUS_ARRAY_NUM] = 0;
        cmd_reg[MODBUS_ARRAY_NUM] = 0;
        gain_reg[MODBUS_ARRAY_NUM] = 0;
        if (i < 4) {
            mbus_timestamp[i] = 0;
        }
    }
    gRatio = 1.0 * gNumerator / gDenominator;
    mb = modbus_new_tcp(MAESTRO_IP, MODBUS_TCP_DEFAULT_PORT);
    modbus_set_slave(mb, 1);
    if (modbus_connect(mb) == -1) {
        cerr << "connection error : IP=" << MAESTRO_IP << "\n";
        return;
    }
    int ret = modbus_read_registers(mb, pos_straged_id_mbus, 6, tab_reg);
    if (ret < 0) {
        cerr << "connection error : IP=" << MAESTRO_IP << "\n";
        connection_is_ok = false;
        return;
    } else {
        cout << "connect to :" << MAESTRO_IP << endl;
        connection_is_ok = true;
    }
    pos0_motorInpt = read_32bit_from_mbus16bit(tab_reg, 0);// Encoder value of motor INPUT-AXIS
    cout << "init_pos_of_elmo inpt axis :" << pos0_motorInpt << endl;
    enc0_slave_rad_is_loaded = false;
}


int AxisInterface::read_32bit_from_mbus16bit(uint16_t *reg, int startRef) {
    int hi = (reg[startRef] << 16) >> 16;
    int lo = (reg[startRef + 1] << 16) >> 16;
    int ret = hi * 32767 + lo;
    return ret;
}

void AxisInterface::send_32bit_to_mbus16bit(double arg_value, uint16_t *regArr_of_mbus, int startRef) {
    // Convert double(64 bit) to int(32 bit) .... LREAL to DINT
    int int_tmp_int = int(arg_value);
    regArr_of_mbus[startRef] = (int_tmp_int / 32767) & 0xffff;
    regArr_of_mbus[startRef + 1] = (int_tmp_int - regArr_of_mbus[startRef] * 32767) & 0xffff;
}

AxisInterface::~AxisInterface() {
    modbus_close(mb);
}

void AxisInterface::set_ElmoCmd(int TorCmd, int PosCmd, uint16_t *reg) {
    send_32bit_to_mbus16bit(TorCmd, reg, 0);
    send_32bit_to_mbus16bit(PosCmd, reg, 2);
}

void AxisInterface::send_ElmoGains_times10000(double kp_pos, double kp_vel, double ki_vel, int mbus_param_start_id) {
    int kp_pos_int = int(kp_pos * 100.);
    int kp_vel_int = int(kp_vel * 10000000.);
    int ki_vel_int = int(ki_vel * 100.);
    send_32bit_to_mbus16bit(kp_pos_int, gain_reg, 0);
    send_32bit_to_mbus16bit(kp_vel_int, gain_reg, 2);
    send_32bit_to_mbus16bit(ki_vel_int, gain_reg, 4);
    modbus_write_registers(mb, mbus_param_start_id, 6, gain_reg);
//    cout << kp_vel_int << ", " << ki_vel_int;
}


void AxisInterface::send_cmd(float rad_edata, int Torlim) {
    if (enc0_slave_rad_is_loaded) {
        PosCmd = int(direction_mode_plus_or_minus * (4095. * (rad_edata - enc0_slave_rad) / (2.0 * M_PI)) * gRatio +
                     pos0_motorInpt);  // [cnt]
        if (abs(PosCmd - pos) < gRatio * 1.0) {
            // For noise avoidance
            cout << "a_region ";
            PosCmd = pos;
        }
        cout << "ge" << gRatio;
        cout << "pos" << direction_mode_plus_or_minus;
        cout << "/ and result signal"
             << direction_mode_plus_or_minus * (4095. * (rad_edata - enc0_slave_rad) / (2.0 * M_PI)) * gRatio;
        cout << "/ master_enc (raw)" << rad_edata << ", ";
        cout << "master_enc (process)" << rad_edata - enc0_slave_rad << ", ";
        cout << "posComd : " << PosCmd << " Torlim : " << Torlim << endl;
        set_ElmoCmd(Torlim, PosCmd, cmd_reg);
        modbus_write_registers(mb, Torlim_straged_id_mbus, 4, cmd_reg);
    } else {
        cerr << "Set initial encoder value by AxisInterface.init_encSlaveEncoder(double edata)" << endl;
    }
}


void AxisInterface::update_sensor(bool option_mode, int int_mbus_id_optionVal) {
    modbus_read_registers(mb, pos_straged_id_mbus, 6, tab_reg);
    pos = read_32bit_from_mbus16bit(tab_reg, 0);
    vel = read_32bit_from_mbus16bit(tab_reg, 2);
    cur = read_32bit_from_mbus16bit(tab_reg, 4);
    modbus_read_registers(mb, time_straged_id, 8, tab_reg);
    for (int i = 0; i < 4; i++) {
        mbus_timestamp[i] = read_32bit_from_mbus16bit(tab_reg, 2 * i);
    }
    if (option_mode) {
        // Show tor orders
        modbus_read_registers(mb, int_mbus_id_optionVal, 4, tab_reg);
        VelOrder = read_32bit_from_mbus16bit(tab_reg, 0);
        TorOrder = read_32bit_from_mbus16bit(tab_reg, 2);
        //
    }
}

int AxisInterface::get_pos() {
    return pos;
}

int AxisInterface::get_vel() {
    return vel;
}

int AxisInterface::get_cur() {
    return cur;
}

int AxisInterface::get_PosCmd() {
    return PosCmd;
}

double AxisInterface::get_motorOtptAxis_rad() {
    return (pos - pos0_motorInpt) * (2.0 * M_PI / 4095.) / gRatio + enc0_slave_rad;
}

void AxisInterface::init_encSlaveEncoder(double edata, int direction_mode) {
    // Memorize initial angle of slave
    enc0_slave_rad = edata;
    // Set the direction of elmo encoder
    // 1 ... the positive direction of motor-encoder(elmo) and load-encoder is same
    // -1 ... the positive direction of motor-encoder(elmo) and load-encoder is different
    if (direction_mode == 1) {
        direction_mode_plus_or_minus = 1;
    } else if (direction_mode == -1) {
        direction_mode_plus_or_minus = -1;
    } else {
        cerr << "Set correct direction mode (int-style 1 or -1)\n" <<
             "1 = positive ... the positive direction of Elmo and Load is SAME\n"
             << "-1 = negative ... the positive direction of Elmo and Load is DIFFERENT\n"
             << endl;
        return;
    }
    enc0_slave_rad_is_loaded = true;
    cout << "Initial encoder value is loaded:" << edata << endl;
}

void AxisInterface::get_timestamp(int *int_4elem_array) {
    for (int i = 0; i < 4; i++) {
//        cout << mbus_timestamp[i];
        int_4elem_array[i] = mbus_timestamp[i];
    }
}

int AxisInterface::get_VelOrder() {
    return VelOrder;
}

int AxisInterface::get_TorOrder() {
    return TorOrder;
}
