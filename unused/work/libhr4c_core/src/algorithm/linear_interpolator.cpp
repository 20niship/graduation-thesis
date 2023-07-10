#include <cmath>
#include <spdlog/spdlog.h>
#include "algorithm/linear_interpolator.h"

using namespace hr4c::algorithm;

void LinearInterpolator::interpolate(double start, double end, double goal_time, double tick,
                                     std::vector<double>& result) {
    if (goal_time == 0) {
        spdlog::error("invalid goal_time: 0");
        return;
    }
    auto interpolation_times = static_cast<int>(floor(goal_time / tick));
    result.clear();

    for (auto i = 0; i < interpolation_times; ++i) {
        result.emplace_back(start + (end - start) * i / interpolation_times);
    }

    if (result.back() != end) {
        result.emplace_back(end);
    }
}

