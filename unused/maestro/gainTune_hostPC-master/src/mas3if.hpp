#ifndef __MAS3_USB__
#define __MAS3_USB__

#include <iostream>
#include <string>
#include <usb.h>
#include <vector>
#include <fstream>
#include <sstream>  // ※istringstreamを使うために必要

//#define ZERO_POINT_CSV_COLUMNS 3
#define RECODE_LIM_DEGREE 350

using namespace std;

class Mas3USB {
private:
    usb_dev_handle *udh;
    bool debug_mode;
    bool set_zero_flag;
    vector<float> enc_data;
    vector<float> offset_memory;
    vector<int> direction_mode;        // add 20190801
    vector<float> prev_enc_data;       // add 20190713
    vector<float> vel_data;            // add 20190713
    vector<int> sin_data;
    vector<int> cos_data;
    int timestamp = 0;
    vector<int> rotation_cnts;         // add 20190713

    // int rotaion_cnt=0;              // add 20190713
    vector<string> split_csv_lines(string &input, char delimiter);

    float shift_enc_by_zeropoint(float zero_point, float raw_value, bool non_negative_out = true);

    float read_enc_basedOn_zeropoint(float zero_point, int direction_mode, float raw_value);

public:
    Mas3USB(string name);

    ~Mas3USB();

    bool recv_packet();

    vector<float> get_data();

    vector<float> get_prev_data(); // add 20190713

    vector<float> get_velocity();

    vector<int> get_rotaion_cnt(); // add 20190713

    vector<int> get_sin_data();

    vector<int> get_cos_data();

    void set_zeroPoints(string zeroCsvfname, int startIdx, bool show_read_csv_name = true);

    vector<float> get_data_calibrated();

    void debug();

    void setValToVector(vector<float> arg_vec, int start_idx);
};


#endif
