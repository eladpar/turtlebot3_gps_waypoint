#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>

#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES (PI / 180)
#define RADIANES_GRADOS (180 / PI)

double normalizeBearing(double angle);
double normalizeAngleDiff(double angle);
double calcBearing(double lon, double lat, double lon2, double lat2);
double calcDistance(double lon, double lat, double lon2, double lat2);

#endif // UTILS_HPP
