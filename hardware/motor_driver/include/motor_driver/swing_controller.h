#ifndef SWING_CONTROLLER_H
#define SWING_CONTROLLER_H

#include "geometry_msgs/Point.h"
#include "cosmo/state.h"
#include "motor_driver/MotorCommand.h"

namespace motor_driver
{
    struct SwingConfig
    {
        SwingConfig() : : alpha(0.0), stance_ticks(0), beta(0.0), z_clearance(0.0), default_stance(std::vector<geometry_msgs::Point>);
        
        SwingConfig(double alpha_, unsigned int stance_ticks_, double dt_, double beta_, double z_clearance_,
            std::vector<geometry_msgs::Point> default_stance_) 
            : alpha(alpha_), stance_ticks(stance_ticks), beta(beta_), z_clearance(z_clearance_), default_stance(default_stance_)
        {};
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
        SwingController();
        
        /*
        * Constructor sets swing config member variable
        */
        SwingController(motor_driver::SwingConfig config);
        
        /*
        * Function to calculate next foot location for the given leg
        *
        * @param[in] swing_phase - indicates point in progression through swing
        * @param[in] leg_index - index of leg for which to calculate position
        * @param[in] state - current state of cosmo
        * @param[in] command - movement command for cosmo
        *
        * @returns point - next location for foot
        */
        geometry_msgs::Point next_foot_location(const double swing_phase, const unsigned int leg_index, cosmo::State state, const MotorCommand command, bool triangular = true);

        private:

        /*
        * Function to calculate height of current point in swing, triangular swing by default
        *
        * @param[in] swing_phase - current point progression through swing phase
        * @param[in] triangular - indicates that shape of swing should be triangular
        * @param[in] swing height - height of swing
        */
        double swing_height(const double swing_phase, bool triangular = true);

        /*
        * Function calculates the touchdown location of leg
        *
        * @param[in] leg_index - index of leg for which to calculate touchdown location
        * @param[in] command - motion command for cosmo
        */
        geometry_msgs::Point raibert_touchdown_location(const unsigned int leg_index, const MotorCommand command);

        // holds swing config variables ()
        motor_driver::SwingConfig config_;


    }; //end swing controller
}

#endif