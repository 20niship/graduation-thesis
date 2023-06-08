#include <string.h>
#include "mas3if.hpp"
#include <math.h>


Mas3USB::Mas3USB(string name) {
    set_zero_flag = false;
    const char *name_char = name.c_str();
    usb_init();
    usb_find_busses();
    usb_find_devices();
    struct usb_bus *busses;
    busses = usb_get_busses();
    struct usb_bus *bus;
    for (bus = busses; bus; bus = bus->next) {
        struct usb_device *dev;
        for (dev = bus->devices; dev; dev = dev->next) {
            if (dev->descriptor.idVendor == 0x04b4) {
                if (dev->descriptor.idProduct == 0x1004) {
                    usb_dev_handle *udh_tmp = usb_open(dev);
                    char buf[256];
                    usb_get_string_simple(udh_tmp, 2, buf, 256);
//                    std::cout << buf << std::endl;
                    if (strcmp(buf, name_char) == 0) {
//                        if (strcmp(buf, "MAS3-INTERFACE0") == 0) {
                        std::cout << "Encoder Board:" << buf << " ... Detected!" << std::endl;
                        udh = udh_tmp;
                        usb_claim_interface(udh, 0);
                        break;
                    } else {
                        usb_close(udh_tmp);
                        udh = NULL;
                    }
                }
            }
        }
    }
    debug_mode = false;
    char tmp[1];
    tmp[0] = 0x22;
    while (usb_interrupt_write(udh, 6, tmp, sizeof(tmp), 2) < 0);
    enc_data.resize(4, 0);
    prev_enc_data.resize(4, 0);
    sin_data.resize(4, 0);
    cos_data.resize(4, 0);
    rotation_cnts.resize(4, 0);
}


Mas3USB::~Mas3USB() {
    char tmp[1];
    if (udh != NULL) {
        usb_release_interface(udh, 0);
        usb_close(udh);
    }
}

bool
Mas3USB::recv_packet() {
    bool ret = false;
    char tmp[65];
    prev_enc_data = enc_data; // Set prev_enc_data
    if (udh != NULL) {
        int cnt = usb_interrupt_read(udh, 2, tmp, sizeof(tmp), 0);
//        cout << cnt << endl;
        if (cnt > 0) {
            timestamp = (tmp[0] & 0xff);
            for (int j = 0; j < 4; j++) {
                int mask = (1 << (2 * j));
                int mask2 = (1 << (2 * j + 1));
                vector<int> ad_data(4);
                for (int k = 0; k < 4; k++) {
                    int tmp1 = 0;
                    int tmp2 = 0;
                    for (int i = 0; i < 16; i++) {
                        int v = (1 << (15 - i));
                        if ((tmp[16 * k + i + 1] & mask) > 0) {
                            tmp1 += v;
                        }
                        if ((tmp[16 * k + i + 1] & mask2) > 0) {
                            tmp2 += v;
                        }
                    }
                    ad_data[k] = tmp1;
                    if (debug_mode)
                        std::cout << timestamp << " " << j << " " << k << ": " << tmp1 << " " << tmp2 << std::endl;
                }
                sin_data[j] = ad_data[0] - ad_data[1];
                cos_data[j] = ad_data[2] - ad_data[3];
                enc_data[j] = atan2(sin_data[j], cos_data[j]);
                ret = true;
            }

        }
    }
    return ret;
}

vector<float>
Mas3USB::get_data() {
    return enc_data;
}

vector<float>
Mas3USB::get_prev_data() {
    return prev_enc_data;
}

vector<float>
Mas3USB::get_velocity() {
    return enc_data;
}

vector<int>
Mas3USB::get_rotaion_cnt() {
    return rotation_cnts;
}


vector<int>
Mas3USB::get_sin_data() {
    return sin_data;
}

vector<int>
Mas3USB::get_cos_data() {
    return cos_data;
}

void
Mas3USB::debug() {
    debug_mode = true;
}

void Mas3USB::setValToVector(vector<float> arg_vec, int start_idx) {
    for (int i = 0; i < 4; i++) {
        arg_vec[start_idx + i] = enc_data[i];
    }

}

void Mas3USB::set_zeroPoints(string zeroCsvfname, int startIdx, bool show_read_csv_name) {
    offset_memory.resize(4, 0);
    direction_mode.resize(4, 0);
    string line;
    ifstream ifs(zeroCsvfname.c_str());
    if (ifs.fail()) {
        std::cerr << "Failed to open file." << std::endl;
    } else {
        if (show_read_csv_name) {
            cout << "Opened :" << zeroCsvfname << endl;
        }
    }
    int line_cnt = 0;
    int memory_cnt = 0;
    while (getline(ifs, line)) {
        vector<string> strvec = split_csv_lines(line, ',');
//        for (int i = 0; i < 3; i++) {
//            cout << stof(strvec[i]) << "jb";
//        }
        if (startIdx <= line_cnt && line_cnt < startIdx + 4) {
            vector<string> strvec = split_csv_lines(line, ',');
            offset_memory[memory_cnt] = stof(strvec[1]);
            direction_mode[memory_cnt] = stoi(strvec[2]);
//            cout << offset_memory[memory_cnt] << endl;
            memory_cnt += 1;
        }
        line_cnt += 1;
    }
    set_zero_flag = true;
}

vector<string> Mas3USB::split_csv_lines(string &input, char delimiter) {
    istringstream stream(input);
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

float Mas3USB::shift_enc_by_zeropoint(float zero_point, float raw_value, bool non_negative_out) {
    float y = 0;// -M_PI <= y <= M_PI
    if (zero_point > 0) {
        if (zero_point - M_PI < raw_value && raw_value <= M_PI) {
            // y = (x - zero)
            y = raw_value - zero_point;
        } else {
            // y = (x - zero + 2pi )
            y = raw_value - zero_point + 2 * M_PI;
        }
    } else {
        if (-1. * M_PI < raw_value && raw_value <= M_PI + zero_point) {
            // y = (x - zero)
            y = raw_value - zero_point;
        } else {
            // y = (x - zero - 2pi )
            y = raw_value - zero_point - 2 * M_PI;
        }
    }
    if (non_negative_out) {
        y += M_PI;
    }
    return y;
}

vector<float> Mas3USB::get_data_calibrated() {
    vector<float> tmp;
    tmp.resize(4, 0);
    if (set_zero_flag) {
        for (int i = 0; i < 4; i++) {
//        cout << enc_data[i] << "   ";
            tmp[i] = read_enc_basedOn_zeropoint(offset_memory[i], direction_mode[i], enc_data[i]);
        }
    } else {
        cerr << "Call <Mas3USB>.set_zeroPoints before call this function\n";
    }
    return tmp;
}

float Mas3USB::read_enc_basedOn_zeropoint(float zero_point, int direction_mode, float raw_value) {
    float y = 0;
    if (direction_mode == 0) {
        // positive mode : 0 <= y <= 2pi
        if (zero_point < raw_value && raw_value <= M_PI) {
            y = raw_value - zero_point;                          // y = x - zero
        } else {
            y = raw_value - zero_point + 2 * M_PI;

        }
        // Avoid chattering
        if (y > 2.0 * M_PI * RECODE_LIM_DEGREE / 360.) {
            y = 0.;
        }

    } else if (direction_mode == 1) {
        // negative mode : -2pi <= y <= 0
        if (-1 * M_PI < raw_value && raw_value <= zero_point) {
            y = raw_value - zero_point;
        } else {
            y = raw_value - zero_point - 2 * M_PI;
        }
        // Avoid chattering
        if (y < -2.0 * M_PI * RECODE_LIM_DEGREE / 360.) {
            y = 0.;
        }


    } else {
        cout << "direction mode error !! " << endl;
        y = zero_point;
    }
    return y;
}



