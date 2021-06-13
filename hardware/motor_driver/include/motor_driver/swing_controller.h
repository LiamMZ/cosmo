#ifndef SWING_CONTROLLER_H
#define SWING_CONTROLLER_H

namespace motor_driver
{
    struct SwingConfig
    {
        double alpha;
        unsigned int stance_ticks;
        double dt;
        double beta;
        double z_clearance;

    }; // end swing config


    class SwingController
    {
        public:
        SwingController(SwingConfig config);

        private:
        SwingController config_;

    }; //end swing controller
}

#endif