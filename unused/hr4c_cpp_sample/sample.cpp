#include <libhr4c_comm.h>
#include <iostream>


int main() {
    auto instance_no = hr4capi_open(const_cast<char*>("172.16.1.24"),
                                    54321,
                                    1);
    if (instance_no >= 0) {
        hr4capi_start(instance_no);

        double joint_angles[6];
        hr4capi_get_joint_angle(instance_no, joint_angles);
        for (auto i = 0; i < 6; ++i) {
            std::cout << "j" << i << ": "<< joint_angles[i] << std::endl;
        }

        hr4capi_stop(instance_no);
        hr4capi_close(instance_no);
    }

    return 0;
}
