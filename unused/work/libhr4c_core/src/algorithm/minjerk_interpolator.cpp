#include <cmath>
#include <spdlog/spdlog.h>
#include "algorithm/minjerk_interpolator.h"

using namespace hr4c::algorithm;

void MinJerkInterpolator::interpolate(double start, double end, double goal_time, double tick,
                                      std::vector<double>& result) {
    if (goal_time == 0) {
        spdlog::error("invalid goal_time: 0");
    }
    auto interpolation_times = static_cast<int>(floor(goal_time / tick));
    result.clear();

    for (auto i = 0; i < interpolation_times; ++i) {
        auto time_ratio = (tick * i) / goal_time;
        auto coef = (10 * pow(time_ratio, 3) - 15 * pow(time_ratio, 4) + 6 * pow(time_ratio, 5));
        result.emplace_back(start + (end - start) * coef);
    }

    if (result.back() != end) {
        result.emplace_back(end);
    }
}
