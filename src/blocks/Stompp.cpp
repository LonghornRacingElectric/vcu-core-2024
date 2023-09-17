#include "Stompp.h"

/**
 * STOMPP: Software To Omit Multi-Pedal Presses
 * Also known as EV 5.7, but that's a lame name
 *
 * Read the rules!
 *
 * STOMPP must monitor both BSE (Brake System Encoder) AND APPS signals larger than 25%
 * If both of these conditions are true, the following must occur
 * - Power to motors must be shut off
 *      To get out of this:
 *      - APPS signal must fall under 5%
 */

void Stompp::evaluate(VcuParameters *params, StomppInput *input, StomppOutput *output, float deltaTime) {

    // TODO implement rule EV 5.7

    //STOMPP true = motor shutdown
    //STOMPP false = car is operating normally

    //Assume STOMPP is not active
    output->ok = false;

    //Check for two main requirements. Enable STOMPP IF:
    //  Brakes are active
    //  Accelerator Travel > 25%
    if((input->bse > 0) && (input->apps > 25)){     /**Will inactive brake pedal % remain at 0? or will it jitter?**/
        output->ok = true;                          //If this^ isn't addressed, car will shut down everytime apps > 25

        //To disable STOMPP, apps must fall under 5
        while(output->ok)
            if(input->apps < 5)
                output->ok = false;
    }
}

//Does power shutdown happen in STOMPP? Or does that happen in VcuCore/Model?
//STOMPP true = motor shutdown... is this correct?
//Address brake jitter question
