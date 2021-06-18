#ifndef SWING_CONTROLLER_H
#define SWING_CONTROLLER_H

#include "geometry_msgs/Point.h"
#include "cosmo/state.h"
#include "motor_driver/MotorCommand.h"

namespace motor_driver
{
    struct SwingConfig
    {
        double alpha;
        unsigned int stance_ticks;
        double dt;
        double beta;
        double z_clearance;
        std::vector<geometry_msgs::Point> default_stance;

    }; // end swing config


    class SwingController
    {
        public:
        SwingController(SwingConfig config);
        
        geometry_msgs::Point next_foot_location(const double swing_prop, const unsigned int leg_index, cosmo::State state, const MotorCommand command);

        private:
        double swing_height(const double swing_phase, bool triangular = true);

        geometry_msgs::Point raibert_touchdown_location(const unsigned int leg_index, const MotorCommand command);

        SwingController config_;

    }; //end swing controller
}

#endif