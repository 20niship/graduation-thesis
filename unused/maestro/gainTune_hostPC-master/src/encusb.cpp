
#include <string>
#include "encusb.hpp"


EncUSB::EncUSB() {
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
                    string tmps(buf);
                    std::cout << tmps << std::endl;
                    if (tmps.find("ENC-USB-MTG-R") != string::npos) {
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
#if 1
    char tmp[1];
    tmp[0] = 0x22;
    while (usb_interrupt_write(udh, 6, tmp, sizeof(tmp), 2) < 0);
#endif
    enc_data.resize(8, 0);
    std::cout << "initialize finish\n";
}


EncUSB::~EncUSB() {
    char tmp[1];
    if (udh != NULL) {
        usb_release_interface(udh, 0);
        usb_close(udh);
    }
}

bool
EncUSB::recv_packet() {
    bool ret = false;
    char tmp[40];

    if (udh != NULL) {
        int cnt = usb_interrupt_read(udh, 2, tmp, sizeof(tmp), 0);

        if (cnt > 0) {
            bool check = true;
            for (int i = 0; i < 20; i++) {
                if ((tmp[2 * i] & 0x1f) != i)
                    check = false;
            }
            int tmp2 = ((tmp[0] >> 5) & 0x7);
            for (int i = 0; i < 20; i++) {
                int tmp3 = ((tmp[2 * i] >> 5) & 0x7);
                if (tmp2 != tmp3)
                    check = false;
            }
            if (check) {
                ret = true;
                for (unsigned int i = 0; i < enc_data.size(); i++) {
                    enc_data[i] = 0;
                }
                for (int j = 0; j < 8; j++) {
                    int mask = (1 << j);
                    for (int i = 1; i < 19; i++) {
                        if ((tmp[2 * i + 1] & mask) > 0) {
                            enc_data[j] |= (1 << (18 - i));
                        }
                    }
                }
            }
        }
    }
    return ret;
}

vector<int>
EncUSB::get_data() {
    return enc_data;
}

void
EncUSB::debug() {
    debug_mode = true;
}
