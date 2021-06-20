#ifndef DRIVER_H
#define DRIVER_H

#include <cosmo/state.h>
#include <motor_driver/MotorCommand.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace motor_driver
{
    class Driver
    {
        public:
        Driver(DriverConfig config);
        void run(cosmo::State& state, const MotorCommand command);
        void set_pose_to_default(cosmo::State& state);

        private:
        std::vector<geometry_msgs::Point> step_gait(const cosmo::State& state, const MotorCommand command, std::vector<bool>& contact_modes);

    };
}

#endif