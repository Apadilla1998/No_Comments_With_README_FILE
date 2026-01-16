#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>

inline double clampD(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double clampPct(double v) {
    return clampD(v, -100.0, 100.0);
}

inline double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

inline double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

inline double wrap180(double deg) {
    while (deg > 180.0)  deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

#endif