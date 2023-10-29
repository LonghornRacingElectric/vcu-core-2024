#include <iostream>
#include "VcuModel.h"

using namespace std;

VcuParameters vcuParameters;
VcuInput vcuInput;
VcuOutput vcuOutput;

VcuModel vcuModel;

float arrI[11] = {0.044f, 0.0398f, 0.0792f, 0.1185f, 0.1576f, 0.1976f, 0.2356f, 0.2745f, 0.3135f, 0.3526f, 0.392f};
float arrO[11] = {0.0044f, 0.0447f, 0.0855f, 0.1267f, 0.1688f, 0.2115f, 0.2553f, 0.2953f, 0.3363f, 0.3838f, 0.4388f};

void initParameters() {
    vcuParameters.steeringWheelToInnerWheel = CurveParameter(0.0f, 90.0f, arrI);
    vcuParameters.steeringWheelToOuterWheel = CurveParameter(0.0f, 90.0f, arrO);
}

bool input_bool() {
    float x;
    cin >> x;
    return x != 0.0f;
}

xyz input_xyz() {
    float x, y, z;
    cin >> x;
    cin >> y;
    cin >> z;
    return xyz {x, y, z};
}

void input() {
    cin >> vcuInput.apps1;
    cin >> vcuInput.apps2;

    cin >> vcuInput.bse1;
    cin >> vcuInput.bse2;

    cin >> vcuInput.steeringWheelPotVoltage;

    cin >> vcuInput.wheelDisplacementFl;
    cin >> vcuInput.wheelDisplacementFr;
    cin >> vcuInput.wheelDisplacementBl;
    cin >> vcuInput.wheelDisplacementBr;

    cin >> vcuInput.motorTemp;
    cin >> vcuInput.inverterTemp;
    cin >> vcuInput.batteryTemp;

    cin >> vcuInput.batterySoc;
    vcuInput.inverterReady = input_bool();

    vcuInput.driveSwitch = input_bool();

    cin >> vcuInput.gpsLat;
    cin >> vcuInput.gpsLong;
    cin >> vcuInput.gpsSpeed;
    cin >> vcuInput.gpsHeading;

    vcuInput.imu1Accel = input_xyz();
    vcuInput.imu2Accel = input_xyz();
    vcuInput.imu3Accel = input_xyz();
    vcuInput.imu1Gyro = input_xyz();
    vcuInput.imu2Gyro = input_xyz();
    vcuInput.imu3Gyro = input_xyz();
}

void output_bool(bool b) {
    cout << (int)b << '\n';
}

void output_xyz(xyz vec) {
    cout << vec.x << '\n';
    cout << vec.y << '\n';
    cout << vec.z << '\n';
}

void output_float(float f) {
    cout << f << '\n';
}

void output() {
    output_bool(vcuOutput.enableInverter);
    output_float(vcuOutput.inverterTorqueRequest);

    output_bool(vcuOutput.prndlState);
    output_bool(vcuOutput.r2dBuzzer);

    output_bool(vcuOutput.enableDragReduction);

    output_xyz(vcuOutput.vehicleDisplacement);
    output_xyz(vcuOutput.vehicleVelocity);
    output_xyz(vcuOutput.vehicleAcceleration);

    output_bool(vcuOutput.faultApps);
    output_bool(vcuOutput.faultBse);
    output_bool(vcuOutput.faultStompp);
    output_bool(vcuOutput.faultSteering);

    cout << flush;
}

int main() {
    initParameters();
    vcuModel.setParameters(&vcuParameters);

    while(1) {
        input();
        vcuModel.evaluate(&vcuInput, &vcuOutput, 0.003f);
        output();
    }
}