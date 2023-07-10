#include "utils/hr4c_utils.h"

using namespace hr4c::utils;

void hr4c::utils::SetJointArrayToTrajectory(vector<vector<double>>& joint_array,
                                            const shared_ptr<deque<vector<double>>>& ref_array) {
    for (auto i = 0; i < joint_array[0].size(); ++i) {
        vector<double> targets{};
        targets.reserve(joint_array.size());
        for (auto& j : joint_array) {
            targets.emplace_back(j[i]);
        }
        ref_array->emplace_back(targets);
    }
}

vector<string> hr4c::utils::Split(const string& str, char delim) {
    vector<string> elems;
    string item;
    for (char ch: str) {
        if (ch == delim) {
            if (!item.empty()) {
                elems.emplace_back(item + delim);
            }
            item.clear();
        } else {
            item += ch;
        }
    }
    if (!item.empty()) {
        elems.push_back(item + delim);
    }

    return elems;
}
