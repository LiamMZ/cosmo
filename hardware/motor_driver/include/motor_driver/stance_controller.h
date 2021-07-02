#ifndef STANCE_CONTROLLER_H
#define STANCE_CONTROLLER_H

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include "motor_driver/MotorCommand.h"
#include "cosmo/state.h"

namespace motor_driver
{
    class StanceController
    {
        public:
            StanceController();

            /*
            * Constructor loads configuration variables
            *
            * @param[in] z_time_constant - 
            * @param[in] dt - time increment between time steps/ticks
            */
            StanceController(const double z_time_constant, const double dt);

            /*
            * Function to calculate next foot location based on commands
            * 
            * @param[in] leg_index - index of leg for which position is being calculated 
            * @param[in] state - current state of robot
            * @param[in] command - movement command
            *
            * @returns position of foot for next time step
            */
            geometry_msgs::Point next_foot_location(const unsigned int leg_index, const cosmo::State state, const MotorCommand command);
        private:
            /*
            * Function to calculate transform increment for foot position
            *
            * @param[in] z - current z position of foot
            * @param[in] height - height of robot
            * @param[in] command - movement command
            * @param[out] translation - foot translation delta
            * @returns Transform - transformation about yaw axis
            */
            tf::Transform position_delta(const double z, const double height, const MotorCommand command, geometry_msgs::Point& translation);

            // time increment between time steps/ticks
            double dt_;

            // 
            double z_time_constant_;
    };
}
#endif