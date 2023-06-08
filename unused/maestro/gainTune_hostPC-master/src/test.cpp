#include <iostream>
#include <iomanip>
#include "encusb.hpp"
#include "mas3if.hpp"
#include "filter.hpp"
#include <math.h>
#include <stdio.h>
#include "test.hpp"
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include <string.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <sys/time.h>

// kbhit
#include <termios.h>


using namespace std;
namespace fs=boost::filesystem;
static struct termios init_tio;


//double KP_POS = 240;
//double KP_VEL = 0.18 * pow(10, -5);
//double KI_VEL = 3;
//int TOR_LIM = 2000;
//

//double KP_POS = 200;
//double KP_VEL = 0.139 * pow(10, -5);
//double KI_VEL = 3.6;
//int TOR_LIM = 2500;

int main(int argc, char *argv[]) {

    int gNum = atoi(argv[2]);
    int gDen = atoi(argv[3]);
    double KP_POS = atof(argv[4]);
    double KP_VEL = atof(argv[5]) * pow(10, -5);
    double KI_VEL = atof(argv[6]);
    int TOR_LIM = atof(argv[7]);

//    for (int l = 1; l < 8; l++) {
//        cout << "arg" << l << ":" << argv[l] << ", ";
//    }
//    cout << endl;

    // Axis interface
    cout << eAx1 << "," << gNum << ", " << gDen << ", " << eCom1 << endl;
    AxisInterface ax1(eAx1, gNum, gDen, eCom1);

    e_e_pair Axis_pair[USE_AXIS_NUM] = {
            {"a01", ax1, 6, 0, eAx1, eCom1, TOR_LIM, 1},
//            {"a04", ax1, 7, 1, eAx1, eCom1, TOR_LIM, 1},
//            {"a02", ax1, 9, 2, eAx1, eCom1, TOR_LIM, 1},
//            {"a03", ax1, 8, 3, eAx1, eCom1, TOR_LIM, -1},
//            {"a05", ax1, 11, 5, eAx1, eCom1, TOR_LIM, 1},
//            {"a06", ax1, 10, 4, eAx1, eCom1, TOR_LIM, -1},
    };


    if (argc != 8) {
        cout << "Error! " << endl;
        cout << "Set [Axis ID] [gear Num] [gear Den] [pos_kp] [vel_kp] [vel_ki] [tor_lim]" << endl;
        return 0;
    }
    // Initialize Log writer
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    char dirname[256];
    sprintf(dirname, "%d%02d%02d", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday);
    fs::path dir(dirname);
    fs::create_directory(dir);
    char file[512];
    sprintf(file, "%d%02d%02d/%s_%02d%02d%02d_ver%d.csv", pnow->tm_year + 1900,
            pnow->tm_mon + 1, pnow->tm_mday, argv[1], pnow->tm_hour, pnow->tm_min, pnow->tm_sec, VER);
    std::cout << file << std::endl;
    fs::ofstream fout(file);

    switch (STATE_ID) {
        case Idling: {
            STATE_ID = 1;
        }

        case SetEnc: {
            cout << "Start encoder calibration" << endl;
            encAll.resize(12, 0);
            string zeroPointRecordedFname =
                    string(ZERO_POINT_SAVING_DIR) + string("/") + string(ZERO_POINT_SAVING_FILENAME);
            mas.set_zeroPoints(zeroPointRecordedFname, 0);           // ID=[0, 4) are MAS3-INTERFACE0
            f0.set_para(1, 5);
            f0.set_enc_num(4);
            mas1.set_zeroPoints(zeroPointRecordedFname, 4, false);          // ID=[4, 8) are MAS3-INTERFACE1
            f1.set_para(1, 5);
            f1.set_enc_num(4);
            mas2.set_zeroPoints(zeroPointRecordedFname, 8, false);          // ID=[8, 12) are MAS3-INTERFACE2
            f2.set_para(1, 5);
            f2.set_enc_num(4);

            //////////// Encoder burnout for
            for (int burnout = 0; burnout < ENC_BURNOUT_STEP; burnout++) {
                if (mas.recv_packet()) {
                    enc_calib = mas.get_data_calibrated();
                    f0.setVal(enc_calib);
                    enc_calib = f0.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i] = enc_calib[i];
                    }
                }
                if (mas1.recv_packet()) {
                    enc_calib = mas1.get_data_calibrated();
                    f1.setVal(enc_calib);
                    enc_calib = f1.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i + 4] = enc_calib[i];
                    }
                }
                if (mas2.recv_packet()) {
                    enc_calib = mas2.get_data_calibrated();
                    f2.setVal(enc_calib);
                    enc_calib = f2.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i + 8] = enc_calib[i];
                    }
                }
            }
            //////////// End burnout
            STATE_ID = SetModbus;
        }

        case SetModbus: {
            // Set Slave Encoder Angle
            int detected_axis = initialize_AllSlaveEnc(Axis_pair);
            if (detected_axis == USE_AXIS_NUM) {
                STATE_ID = StartWhileLoop;
            } else {
//                STATE_ID = SetModbus;
                break;
            }
            // TO DO !!!!   Error handling
        }

        case StartWhileLoop: {
            cout << "Start while loop" << endl;
            int ch;
            init_keyboard();
            while (ch != 'q') {
                // Load data from modbus
                get_allElmo_vals(Axis_pair, modbus_time);
                // Read angles from encoder
                ////////////////////////////
                if (mas.recv_packet()) {
                    enc_calib = mas.get_data_calibrated();
                    f0.setVal(enc_calib);
                    enc_calib = f0.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i] = enc_calib[i];
                    }
                }

                if (mas1.recv_packet()) {
                    enc_calib = mas1.get_data_calibrated();
                    f1.setVal(enc_calib);
                    enc_calib = f1.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i + 4] = enc_calib[i];
                    }
                }

                if (mas2.recv_packet()) {
                    enc_calib = mas2.get_data_calibrated();
                    f2.setVal(enc_calib);
                    enc_calib = f2.get_val_vector();
                    for (int i = 0; i < 4; i++) {
                        encAll[i + 8] = enc_calib[i];
                    }
                }

                // Upload all orders (= torLim, target_pos) to modbus
                send_allElmo_vals(Axis_pair, encAll, KP_POS, KP_VEL, KI_VEL);

                /// log writer is activated and write down to csv
                /// if update is detected (defined by "msec" change of modbus_register)
                if (old_modbus_msec != modbus_time[3]) {
                    // Write to logfile ... h, m, s, ms
                    for (int i = 0; i < 4; i++) {
                        fout << modbus_time[i] << ", ";
                    }
                    // Write robot data @ ax[0], ax[1], ....;
                    // ax[n] = { enc_master, enc_slave, ax_pos, ax_vel, ax_cur, ax_cur_order}
                    for (int i = 0; i < USE_AXIS_NUM; i++) {
                        int master_id = Axis_pair[i].masterEncId;
                        int slave_id = Axis_pair[i].slaveEncId;
                        fout << encAll[master_id] << ", " << encAll[slave_id] << ", ";
                        fout << Axis_pair[i].elmo_axis.get_pos() << ", " << Axis_pair[i].elmo_axis.get_vel() << ", "
                             << Axis_pair[i].elmo_axis.get_cur() << ", ";
                        fout << Axis_pair[i].elmo_axis.get_PosCmd() << ", ";
                        // Orders for debug
                        fout << Axis_pair[i].elmo_axis.get_VelOrder() << ", " << Axis_pair[i].elmo_axis.get_TorOrder()
                             << ", ";
                    }
                    fout << endl;
                }
                ///////////////////////////////////
                if (kbhit()) {
                    ch = readkey();
                    usleep(1000);
                    fout.close();
                    break;
                }
            }
//            close_keyboard();
            STATE_ID = TerminationProcess;
        }

        case TerminationProcess: {
            cout << "Terminate process" << endl;
            fout.close();
            close_keyboard();
            break;
        }

        default:
            cout << "Default" << endl;
//            modbus_close(mb);
            fout.close();
            break;
    }
    return 0;
}

void Set_filteredEnc(Mas3USB arg_mas, FILTER arg_filter, vector<float> &arg_enc, int startRef) {
    vector<float> tmp_enc, filtered_enc;
    tmp_enc.resize(4, 0.);
    filtered_enc.resize(4, 0.);
    if (arg_mas.recv_packet()) {
        cout << "Inside of recv_packet()" << endl;
        tmp_enc = arg_mas.get_data_calibrated();
        arg_filter.setVal(tmp_enc);
        filtered_enc = arg_filter.get_val_vector();
        for (int i = 0; i < 4; i++) {
            arg_enc[startRef + i] = filtered_enc[i];
//            cout << arg_enc[startRef + i] << "  ,";
        }
        cout << arg_enc[startRef] << endl;
    }
}

int initialize_AllSlaveEnc(e_e_pair arg_pair[]) {
    int read_enc_num = 0;    // cnt is controlable axis num
    for (int cnt = 0; cnt < USE_AXIS_NUM; cnt++) {
        int slaveEncId_tmp = arg_pair[cnt].slaveEncId;
        int direction_pos_or_nega = arg_pair[cnt].elmo_and_load_enc_Direction_is_same;
        arg_pair[cnt].elmo_axis.init_encSlaveEncoder(encAll[slaveEncId_tmp], direction_pos_or_nega);
        if (arg_pair[cnt].elmo_axis.connection_is_ok) {
            read_enc_num += 1;
        }
    }
    cout << "Read all initial angle of slave encoder:" << read_enc_num << endl;
    return read_enc_num;
}

void get_allElmo_vals(e_e_pair arg_pair[], int time4dArray[]) {
    old_modbus_msec = time4dArray[3];
    for (int cnt = 0; cnt < USE_AXIS_NUM; cnt++) {
        arg_pair[cnt].elmo_axis.update_sensor();
        if (cnt == USE_AXIS_NUM - 1) {
            // timestamp is writen when final axis sensor is updated
            arg_pair[cnt].elmo_axis.get_timestamp(time4dArray);
        }
    }
}

void send_allElmo_vals(e_e_pair arg_pair[], vector<float> &rad_data_vector, double posKP, double velKP, double velKI) {
    for (int cnt = 0; cnt < USE_AXIS_NUM; cnt++) {
        int torLim = arg_pair[cnt].currentLim_mA;
        int master_enc_id = arg_pair[cnt].masterEncId;
        arg_pair[cnt].elmo_axis.send_cmd(rad_data_vector[master_enc_id], torLim);
    }
    // Add for gain tuning
    arg_pair[0].elmo_axis.send_ElmoGains_times10000(posKP, velKP, velKI);
}

void logwite(boost::filesystem::ofstream fileout, string csv_deliminator) {
    // Modbus Timestamp
    for (int i = 0; i < 4; i++) {
        fileout << modbus_time[i] << endl;
    }
    // Write Axis information
}

// kbhit s

void init_keyboard() { tcgetattr(0, &init_tio); }

void close_keyboard() { tcsetattr(0, TCSANOW, &init_tio); }

int kbhit() {
    struct termios tio;
    struct timeval tv;
    fd_set rfds;
    // set up terminal
    memcpy(&tio, &init_tio, sizeof(struct termios));
    tio.c_lflag &= ~(ICANON);
    tcsetattr(0, TCSANOW, &tio);
    // do not wait
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_usec = 0;
    tv.tv_sec = 0;
    select(1, &rfds, NULL, NULL, &tv);
    // back to initial terminal mode
    tcsetattr(0, TCSANOW, &init_tio);
    return (FD_ISSET(0, &rfds) ? 1 : 0);
}

// key input
int readkey() {
    int ch;
    struct termios tio;
    // set up terminal
    memcpy(&tio, &init_tio, sizeof(struct termios));
    tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tio);
    // input key
    read(0, &ch, 1);
    // back to initial terminal mode
    tcsetattr(0, TCSANOW, &init_tio);
    return ch;
}