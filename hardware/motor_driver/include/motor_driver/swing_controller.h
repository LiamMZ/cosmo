#ifndef SWING_CONTROLLER_H
#define SWING_CONTROLLER_H

namespace motor_driver
{
    struct SwingConfig
    {
        double alpha;
        unsigned int stance_ticks;
        unsigned int dt;
        double beta;
        double z_clearance;

    }; // end swing config


    class SwingController
    {
        SwingController(SwingConfig config);
    }; //end swing controller
}

#endif