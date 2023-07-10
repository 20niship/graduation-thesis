#ifndef HR4C_CORE_HR4C_UTILS_H
#define HR4C_CORE_HR4C_UTILS_H
#include <deque>
#include <memory>
#include <vector>

using namespace std;

namespace hr4c::utils {
    void SetJointArrayToTrajectory(vector<vector<double>>& joint_array,
                                   const shared_ptr<deque<vector<double>>>& ref_array);
    vector<string> Split(const string& str, char delim);
} // namespace hr4c::utils
#endif //HR4C_CORE_HR4C_UTILS_H
