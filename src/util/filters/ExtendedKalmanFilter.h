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

/**
 * @brief Represents the state of control variables.
 *
 * This struct stores the values of control variables used in the Extended Kalman Filter.
 * It includes the acceleration in the x and y directions, as well as the angular velocity.
 *
 * @param a_x the acceleration in the x direction
 * @param a_y the acceleration in the y direction
 * @param v_theta the angular velocity (in radians per second)
 */
typedef struct ControlState {
    float a_x;
    float a_y;
    float v_theta;
} ControlState;

/**
 * @brief Struct representing the variables used in the Jacobian matrix calculations.
 *
 * This struct stores the variables required for calculating the Jacobian matrix in the Extended
 * Kalman Filter. It includes the time step (delta_t), acceleration in the x and y directions (a_x
 * and a_y), and the angle (theta).
 *
 * @param delta_t the time step
 * @param a_x the acceleration in the x direction
 * @param a_y the acceleration in the y direction
 * @param theta the angle in radians
 */
typedef struct JacobianVariables {
    double delta_t;
    double a_x;
    double a_y;
    double theta;
} JacobianVariables;

/**
 * @brief Represents the state of a vehicle.
 *
 * This struct stores the position, velocity, and orientation of a vehicle.
 * The position is represented by the coordinates (x, y), the velocity by (v_x, v_y),
 * and the orientation by theta.
 *
 * @param x the x-coordinate of the vehicle
 * @param y the y-coordinate of the vehicle
 * @param v_x the velocity in the x direction
 * @param v_y the velocity in the y direction
 * @param theta the angle of the vehicle in radians
 */
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
    Matrix innovation_state;
    Matrix jacobian;
    VehicleState struct_state;
    ControlState prev_control;

    Matrix covariance_estimate;
    Matrix process_covariance;
    Matrix observation_covariance;

    double x, y, v_x, v_y, a_x, a_y, theta;

    /**
     *  -------------------------
     * | Filter Prediction Steps |
     *  -------------------------
     */

    void predictState(ControlState control, float delta_t);
    Matrix predictCovariance();
    Matrix computeStateTransitionJacobian(JacobianVariables variables);

    /**
     *  -------------------------
     * |   State Update Steps   |
     *  -------------------------
     */

    Matrix getResidualCovariance();
    Matrix getOptimalKalmanGain(const Matrix& residual_covariance);
    Matrix updateStateEstimate(const Matrix& kalman_gain, const Matrix& y);
    Matrix updateCovarianceEstimate(const Matrix& predicted_covariance);

    /**
     *  -------------------------
     * |     Helper Methods     |
     *  -------------------------
     */

    static PositionalState getGeographicUnits(PositionalState* loc, double theta);
    static double getGeographicTheta(double theta);

   public:
    ExtendedKalmanFilter();

    VehicleState getState() { return struct_state; };

    void update(ControlState u, VehicleState z, float delta_time);
};

#endif  // VCU_CORE_EXTENDEDKALMANFILTER_H
