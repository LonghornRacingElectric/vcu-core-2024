//
// Created by Dhairya Gupta on 9/26/23.
//

#ifndef VCU_CORE_POSITION_H
#define VCU_CORE_POSITION_H

#include <cmath>

#include "blocks/TrackPositioning.h"
#include "util/Structs.h"

typedef struct SquareDist {
    double x_displacement;
    double y_displacement;
} SquareDist;

class Position {
   private:
    constexpr static double radiusOfEarthMeters = 6357 * 1000;
    constexpr static double radiusOfEarth = 6357;

   public:
    inline static double degreesToRadians(double deg) { return deg * (M_PI / 180); }

    static xyz convertToXYZ(double longitude, double latitude);

    static double distanceBetween(double lng1, double lat1, double lng2, double lat2);

    static SquareDist unitDistanceBetween(double lng1, double lat1, double lng2, double lat2);

    static TrackGPSLocation convertToLongitudeLatitude(TrackGPSLocation initial_location,
                                                       xyz cartesianCoords);
};

#endif  // VCU_CORE_POSITION_H
