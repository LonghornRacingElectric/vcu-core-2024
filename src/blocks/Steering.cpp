#include "Steering.h"

/**
 * We have a potentiometer on the steering wheel that we're using the measure the steering wheel angle.
 * From the steering wheel angle, we can infer what angle the wheels are at. It's not actually proportional;
 * You should talk to dynamics and ask them how to translate a steering wheel angle to the left and right tire angles.
 *
 * Output the steering wheel angle, the four tire angles, and the boolean ok = true if the potentiometer
 * isn't acting up.
*/

void Steering::evaluate(VcuParameters *params, SteeringInput *input, SteeringOutput *output, float deltaTime) {

    // TODO implement

    output->steeringWheelAngle = 0;

    output->wheelAngleFl = output->steeringWheelAngle * 2.0f;
    output->wheelAngleFr = output->steeringWheelAngle * 2.0f;
    output->wheelAngleBl = 0; // the rears will very likely be 0, but rear wheel steering is permitted by the rules :0
    output->wheelAngleBr = 0;

    output->ok = true;
}
