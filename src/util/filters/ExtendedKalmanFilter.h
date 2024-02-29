//
// Created by Dhairya Gupta on 2/18/24.
//

#ifndef VCU_CORE_EXTENDEDKALMANFILTER_H
#define VCU_CORE_EXTENDEDKALMANFILTER_H

#include "util/math/Matrix.h"

typedef struct PositionalState {
    double x;
    double y;
} PositionalState;

typedef struct ControlState {
    float a_x;
    float a_y;
    float v_theta;
} ControlState;

typedef struct JacobianVariables {
    float delta_t;
    double a_x;
    double a_y;
    double theta;
} JacobianVariables;

typedef struct VehicleState {
    float x;
    float y;
    float v_x;
    float v_y;
    float theta;
} VehicleState;

class ExtendedKalmanFilter {

private:
    Matrix state;
    Matrix state_prev;
    Matrix jacobian;
    VehicleState struct_state;

    double x, y, v_x, v_y, a_x, a_y, theta;

    void f(ControlState control, float delta_t);

     Matrix evaluateJacobian(JacobianVariables variables);

     static PositionalState convertLocalToGlobal(PositionalState *loc, double theta);

     static double getGlobalTheta(double theta);

public:
    ExtendedKalmanFilter();

    VehicleState getState() {
        return struct_state;
    };


    void update(ControlState u, float delta_time);
}

#endif //VCU_CORE_EXTENDEDKALMANFILTER_H
