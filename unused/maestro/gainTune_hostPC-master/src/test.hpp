#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include "AxisInterface.h"
#include <string.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#define VER 0

// Encoder config
#define ZERO_POINT_SAVING_DIR       "/home/takayuki/CLionProjects/enc_calib/build"
#define ZERO_POINT_SAVING_FILENAME      "zerosfas.csv"
#define ENC_BURNOUT_STEP 1000
Mas3USB mas("MAS3-INTERFACE0");
Mas3USB mas1("MAS3-INTERFACE1");
Mas3USB mas2("MAS3-INTERFACE2");
FILTER f0, f1, f2;              //  For 2 encoder board, to avoid memory double free or corruption
vector<float> enc_calib;
vector<float> encAll;

// Mbus config
#define USE_AXIS_NUM 1
enum mBusStartId {
    eTimeStartId = 0,
    eAx1 = 10,
    eKp_p = 16,
    eKp_v = 18,
    eKi_v = 20,
    eCom1 = 22,
    eVorder = 26,
    eTorder = 28,
};

// Structure of axis and encoder
typedef struct Elmo_Enc_pair {
    char name[20];
    AxisInterface elmo_axis;
    int slaveEncId;
    int masterEncId;
    int modbus_fromElmoId;
    int modbus_toElmoId;
    int currentLim_mA;
    int elmo_and_load_enc_Direction_is_same;
} e_e_pair;


int modbus_time[4];
int old_modbus_msec;

void Set_filteredEnc(Mas3USB arg_mas, FILTER arg_filter, vector<float> &arg_enc, int startRef);

int initialize_AllSlaveEnc(e_e_pair arg_pair[]);

void get_allElmo_vals(e_e_pair arg_pair[], int time4dArray[]);

void send_allElmo_vals(e_e_pair arg_pair[], vector<float> &rad_data_vector, double posKP, double velKP, double velKI);

//State Definition
enum StateNumbaer {
    Idling,
    SetEnc,        // Read the encoder board
    SetModbus,        // Read the Modbus activation
    StartWhileLoop,        //
    TerminationProcess
};

int STATE_ID = Idling;


void init_keyboard();

void close_keyboard();

int kbhit();

int readkey();