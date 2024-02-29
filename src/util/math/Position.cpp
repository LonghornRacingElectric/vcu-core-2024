//
// Created by Dhairya Gupta on 9/26/23.
//

#include "Position.h"
#include <cmath>

xyz Position::convertToXYZ(double longitude, double latitude) {
    double x = radiusOfEarthMeters * cos(latitude) * cos(longitude);
    double y = radiusOfEarthMeters * cos(latitude) * sin(longitude);
    double z = radiusOfEarthMeters * sin(latitude);

    return { (float) x, (float) y, (float) z};
}

/**
 * Haversine formula for great circle distance between lat/lng points
 * @param lng1
 * @param lat1
 * @param lng2
 * @param lat2
 * @return
 */
double Position::distanceBetween(double lng1, double lat1, double lng2, double lat2) {
    // distance between latitudes and longitudes
    double dLat = degreesToRadians(lat2 - lat1);
    double dLon = degreesToRadians(lng2 - lng1);

    // convert to radians
    lat1 = degreesToRadians(lat1);
    lat2 = degreesToRadians(lat2);

    // apply formulae
    double a = pow(sin(dLat / 2), 2) +
               pow(sin(dLon / 2), 2) *
               cos(lat1) * cos(lat2);
    double c = 2 * asin(sqrt(a));
    return radiusOfEarth * c;
}

double Position::degreesToRadians(double deg) {
    return deg * (M_PI / 180);
}

/**
 * Assumes that lng1 is the first location, and lng2 is the second. This matters for the direction this vector gives.
 * @param lng1
 * @param lat1
 * @param lng2
 * @param lat2
 * @return
 */
SquareDist Position::unitDistanceBetween(double lng1, double lat1, double lng2, double lat2) {
    // Creates a square using the passed in latitudes and longitudes
    double lat3 = lat1, lng3 = lng2, lat4 = lat2, lng4 = lng1;

    // Calculate dist for each.
    double y_dist = distanceBetween(lng1, lat1, lng3, lat3);
    double x_dist = distanceBetween(lng1, lat1, lng4, lat4);

    SquareDist displacement = {
            x_dist,
            y_dist,
    };

    return displacement;
}
