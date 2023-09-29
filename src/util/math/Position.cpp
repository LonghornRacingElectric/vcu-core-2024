//
// Created by Dhairya Gupta on 9/26/23.
//

#include "Position.h"
#include <cmath>

xyz Position::convertToXYZ(double longitude, double latitude) {
    // TODO -- IMPLEMENT THIS METHOD;
    double x = radiusOfEarthMeters * cos(latitude) * cos(longitude);
    double y = radiusOfEarthMeters * cos(latitude) * sin(longitude);
    double z = radiusOfEarthMeters * sin(latitude);

    return { (float) x, (float) y, (float) z};
}