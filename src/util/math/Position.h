//
// Created by Dhairya Gupta on 9/26/23.
//

#ifndef VCU_CORE_POSITION_H
#define VCU_CORE_POSITION_H


#include "util/Structs.h"

class Position {

private:
    float longitude, latitude;
    constexpr static double radiusOfEarthMeters = 6357 * 1000;

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

    // TODO -- IMPLEMENT USING HAVERSINE
    static xyz convertToXYZ(double longitude, double latitude);

    static Position convertToLongitudeLatitude(xyz cartesianCoords);
};


#endif //VCU_CORE_POSITION_H
