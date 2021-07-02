#ifndef DRIVER_H
#define DRIVER_H

#include <cosmo/state.h>
#include <geometry_msgs/Point.h>

#include <motor_driver/kinematics.h>
#include <motor_driver/MotorCommand.h>
#include <motor_driver/gait_controller.h>
#include <motor_driver/stance_controller.h>
#include <motor_driver/swing_controller.h>
#include <motor_driver/util.h>

#include <ros/ros.h>
#include <unordered_map>
#include <vector>

namespace motor_driver
{

    class Driver
    {
        public:
        Driver(ros::NodeHandle nh);

        void run(const MotorCommand command, cosmo::State& state);
        
        void set_pose_to_default(cosmo::State& state);

        private:
        std::vector<geometry_msgs::Point> step_gait(const MotorCommand command, std::vector<bool>& contact_modes, const cosmo::State& state);
        bool get_gait_params(const std::vector< std::vector<bool> >& contact_phases, unsigned int& num_phases, std::vector<unsigned int>& phase_tics,  unsigned int& phase_length);
        bool get_stance_params(double& z_time_constant, double& dt);
        bool get_swing_params(SwingConfig& swing_config);
        bool get_kinematics_params(double& abduction_offset, double& L1, double& L2, std::vector<geometry_msgs::Point>& leg_origins);
        std::vector< geometry_msgs::Point> rotate_foot_locations(const double roll, const double pitch, const std::vector< geometry_msgs::Point>& foot_locations);
        bool update_parameters();

        ros::NodeHandle nh_;
        SwingController swing_controller_;
        GaitController gait_controller_;
        StanceController stance_controller_;
        Kinematics inverse_kinematics_;

        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState, EnumClassHash> hop_transition_mapping_;
        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState, EnumClassHash> trot_transition_mapping_;
        std::unordered_map< cosmo::BehaviorState, cosmo::BehaviorState, EnumClassHash> activate_transition_mapping_;
        std::vector<geometry_msgs::Point> default_stance_;
        unsigned int swing_ticks_;
        const float MAX_TILT = 0.4;
        const float CORRECTION_FACTOR = 0.8;
        double smoothed_yaw_;
        double default_z_ref_;
        double max_yaw_rate_;
    };
}

#endif