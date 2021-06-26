#ifndef DRIVER_H
#define DRIVER_H

#include <cosmo/state.h>
#include <geometry_msgs/Point.h>
#include <motor_driver/MotorCommand.h>
#include <motor_driver/gait_controller.h>
#include <motor_driver/stance_controller.h>
#include <motor_driver/swing_controller.h>
#include <ros/ros.h>
#include <unordered_map.h>
#include <vector>

namespace motor_driver
{

    class Driver
    {
        public:
        Driver(ros::NodeHandle nh);

        std::vector< std::vector<double> > run(const MotorCommand command);
        
        std::vector< std::vector<double> > set_pose_to_default();

        private:
        std::vector<geometry_msgs::Point> step_gait(const MotorCommand command, std::vector<bool>& contact_modes);
        bool get_gait_params(const std::vector< std::vector<bool> >& contact_phases, unsigned int& num_phases, std::vector<unsigned int>& phase_tics,  unsigned int& phase_length);
        bool get_stance_params(double& z_time_constant, double& dt);
        bool get_swing_params(SwingConfig& swing_config);

        cosmo::State get_state();
        ros::NodeHandle nh_;
        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState> hop_transition_mapping_;
        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState> trot_transition_mapping;
        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState> activate_transition_mapping;
        std::vector<geometry_msgs::Point> default_stance_;
    };
}

#endif