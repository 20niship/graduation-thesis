#ifndef HR4C_CORE_MINJERK_INTERPOLATOR_H
#define HR4C_CORE_MINJERK_INTERPOLATOR_H
#include "algorithm/interpolator.h"

namespace hr4c::algorithm {
    class MinJerkInterpolator : public Interpolator {
    public:
        void interpolate(double start, double end, double goal_time, double tick, vector<double>& result) override;
    };
}
#endif //HR4C_CORE_MINJERK_INTERPOLATOR_H
