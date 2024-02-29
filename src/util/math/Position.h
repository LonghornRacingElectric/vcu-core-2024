//
// Created by Dhairya Gupta on 9/26/23.
//

#ifndef VCU_CORE_POSITION_H
#define VCU_CORE_POSITION_H


#include "util/Structs.h"


typedef struct SquareDist {
    double x_displacement;
    double y_displacement;
} SquareDist;

class Position {

private:
    float longitude, latitude;
    constexpr static double radiusOfEarthMeters = 6357 * 1000;
    constexpr static double radiusOfEarth = 6357;

    static double degreesToRadians(double deg);

public:
    Position(float longitude, float latitude) {
        this->longitude = longitude;
        this->latitude = latitude;
    }

    // TODO - CONVERT TYPE OF XYZ TO LAT/LONG USING HAVERSINE FUNCTION
    explicit Position(xyz coordinates) {
        this->longitude = coordinates.x;
        this->latitude = coordinates.y;
    }

    static xyz convertToXYZ(double longitude, double latitude);

    static double distanceBetween(double lng1, double lat1, double lng2, double lat2);

    static SquareDist unitDistanceBetween(double lng1, double lat1, double lng2, double lat2);

    static Position convertToLongitudeLatitude(xyz cartesianCoords);
};


#endif //VCU_CORE_POSITION_H
