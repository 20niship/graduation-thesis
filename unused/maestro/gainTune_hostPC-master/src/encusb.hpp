#ifndef __ENC_USB__
#define __ENC_USB__

#include <iostream>
#include <string>
#include <usb.h>
#include <vector>

using namespace std;

class EncUSB {
private:
    usb_dev_handle *udh;
    bool debug_mode;
    vector<int> enc_data;
public:
    EncUSB();

    ~EncUSB();

    bool recv_packet();

    vector<int> get_data();

    void debug();
};


#endif
