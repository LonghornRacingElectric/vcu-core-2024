//
// Created by Dhairya Gupta on 10/1/23.
//

#ifndef VCU_CORE_KALMANFILTER_H
#define VCU_CORE_KALMANFILTER_H


#include "util/math/Matrix.h"

class KalmanFilter {


public:
//    /**
//     * Create a Kalman filter with the specified matrices.
//     *   A - System dynamics matrix
//     *   C - Output matrix
//     *   Q - Process noise covariance
//     *   R - Measurement noise covariance
//     *   P - Estimate error covariance
//     */
//    KalmanFilter(
//            double dt,
//            const Matrix& A,
//            const Matrix& C,
//            const Matrix& Q,
//            const Matrix& R,
//            const Matrix& P
//    );

    KalmanFilter();

    KalmanFilter(int variables, float t0, const Matrix &x0, const Matrix &u0, const Matrix &covariance);

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(int variables, float t0, const Matrix& x0, const Matrix& u0, const Matrix& covariance);


    bool isInitialized() {
        return initialized;
    }

    /**
    * Update the estimated state based on measured values.
    */
    void update(const Matrix& y, const Matrix& control, float dt);

    Matrix state() {
        return x_hat;
    }

private:
    bool initialized;
    Matrix x_hat, x_hat_new;

    /* Variables present in the system */
    int n;

    /* Stores the acceleration in all directions of the system */
    Matrix u;

    /* State transition matrix for position and velocity */
    Matrix stateTransition;

    /* Control transition matrix for the control (acceleration) */
    Matrix controlTransition;

    Matrix processCovariance;

    Matrix R;
};


#endif //VCU_CORE_KALMANFILTER_H
