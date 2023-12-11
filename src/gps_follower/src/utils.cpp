#include "utils.hpp"

double normalizeBearing(double angle) {
    while (angle < 0.0) {
        angle += 360.0;
    }

    while (angle >= 360.0) {
        angle -= 360.0;
    }

    return angle;
}
double normalizeAngleDiff(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
}


// Function to calculate the bearing between two GPS coordinates
// The bearing is the angle between the north direction and the direction from one point to another point.
// The function calculates the initial bearing in degrees from the origin to the destination point.
double calcBearing(double lon, double lat, double lon2, double lat2) {
    double teta1 = static_cast<double>(M_PI / 180.0) * lat; // Convert degrees to radians
    double teta2 = static_cast<double>(M_PI / 180.0) * lat2; // Convert degrees to radians
    double delta1 = static_cast<double>(M_PI / 180.0) * (lat2 - lat); // Convert degrees to radians
    double delta2 = static_cast<double>(M_PI / 180.0) * (lon2 - lon); // Convert degrees to radians


    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
    double brng = atan2(y, x);
    brng = static_cast<double>(180.0 / M_PI) * brng; // Convert radians to degrees
    brng = normalizeBearing(brng);
    return brng;
}

// Function to calculate the Harvesine distance between two GPS coordinates
// The Harvesine formula calculates the shortest distance between two points on the surface of a sphere given their longitudes and latitudes.
// It assumes the Earth is a perfect sphere, which leads to small errors at large distances.
double calcDistance(double lon, double lat, double lon2, double lat2)
{
    double  lat_new = lat2 * GRADOS_RADIANES;
    double  lat_old = lat * GRADOS_RADIANES;
    double  lat_diff = (lat-lat2) * GRADOS_RADIANES;
    double  lng_diff = (lon-lon2) * GRADOS_RADIANES;

    double  a = sin(lat_diff/2) * sin(lat_diff/2) +
                cos(lat_new) * cos(lat_old) *
                sin(lng_diff/2) * sin(lng_diff/2);
    double  c = 2 * atan2(sqrt(a), sqrt(1-a));

    double  distance = RADIO_TERRESTRE * c;
        
    return distance;
}