#ifndef HR4C_CORE_INTERPOLATOR_H
#define HR4C_CORE_INTERPOLATOR_H
#include <vector>

namespace hr4c::algorithm {
    using namespace std;

    class Interpolator {
    public:
        virtual ~Interpolator() = default;
        virtual void interpolate(double start, double end, double goal_time, double tick, vector<double>& result) = 0;
    };
}
#endif //HR4C_CORE_INTERPOLATOR_H
