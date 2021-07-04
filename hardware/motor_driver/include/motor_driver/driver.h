#ifndef DRIVER_H
#define DRIVER_H


#include <geometry_msgs/Point.h>

#include <motor_driver/kinematics.h>
#include <motor_driver/MotorCommand.h>
#include <motor_driver/gait_controller.h>
#include <motor_driver/stance_controller.h>
#include <motor_driver/State.h> // state ros message
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

        /*
        * Move State forward one time step
        *
        * @param[in] command - command for movement
        * @param[in] state - current state
        *
        * @param[out] state - state after one time step
        */
        void run(const MotorCommand command, State& state);
        
        /*
        * Function to move joints to default positions
        *
        * @param[in] state - current state of robot
        * @param[out] state - updated state
        */
        void set_pose_to_default(State& state);

        private:
        /*
        * calculate the next foot locations for the step gate
        * @param[in] command - 
        * @param[in] contact_modes - foot contacts for all of the phases
        * @param[in] state - current state of robot
        * 
        * @returns new foot locations
        */
        std::vector<geometry_msgs::Point> step_gait(const MotorCommand command, std::vector<bool>& contact_modes, State& state);

        /*
        * Function to retrieve gait related parameters from the parameter server
        *
        * @param[out] contact_phases - contact settings for all of the phases
        * @param[out] num_phases - the number of phases for the gait
        * @param[out] phase_ticks - number of ticks per phase
        * @param[out] phase_length - 
        * @returns true if parameters loaded successfully
        */
        bool get_gait_params(std::vector< std::vector<bool> >& contact_phases, int& num_phases, std::vector<int>& phase_ticks,  int& phase_length);

        /*
        *  Function to retrieve stance parameters from the parameter server
        *
        * @parame[out] z_time_constant - 
        * @param[out] dt - time per tick
        * @returns true if parameters loaded successfully
        */
        bool get_stance_params(double& z_time_constant, double& dt);

        /*
        * Function to get swing parameters from the parameter server
        *
        * @param[out] swing_config - struct to hold parameters needed for swing controller
        * @returns true if parameters loaded successfully
        */
        bool get_swing_params(SwingConfig& swing_config);

        /*
        * Function to retrieve kinematics related parameters from parameter server
        *
        * @param[out] abduction_offset - offset in y dim for hip axis
        * @param[out] L1 - distance to link L1
        * @param[out] L2 - distance to link L2
        * @param[out] leg_origins - 
        * @returns true if parameters loaded successfully
        */
        bool get_kinematics_params(double& abduction_offset, double& L1, double& L2, std::vector<geometry_msgs::Point>& leg_origins);

        /*
        * Function to rotate locations based on passed in angle values
        *
        * @param[in] roll - angle to rotate in roll dir
        * @param[in] pitch - angle to rotate in pitch dir
        * @param[in] yaw - angle to rotate in yaw dir
        * @param[in] foot_locations - pre-rotation foot locations
        * returns - rotated foot locations
        */
        std::vector< geometry_msgs::Point> rotate_foot_locations(const double roll, const double pitch, const double yaw, const std::vector< geometry_msgs::Point>& foot_locations);
        
        /*
        * Function to update driver class parameters, and parameters for driver controllers
        *
        * @returns true if parameters are successfully updated
        */
        bool update_parameters();

        ros::NodeHandle nh_;
        SwingController swing_controller_;
        GaitController gait_controller_;
        StanceController stance_controller_;
        Kinematics inverse_kinematics_;

        std::unordered_map< uint8_t, uint8_t> hop_transition_mapping_;
        std::unordered_map< uint8_t, uint8_t> trot_transition_mapping_;
        std::unordered_map< uint8_t, uint8_t> activate_transition_mapping_;
        std::vector<geometry_msgs::Point> default_stance_;

        int swing_ticks_;
        const float MAX_TILT = 0.4;
        const float CORRECTION_FACTOR = 0.8;
        double smoothed_yaw_;
        double default_z_ref_;
        double max_yaw_rate_;
        double max_stance_yaw_;
        double max_stance_yaw_rate_;
        double yaw_time_constant_;
        double dt_;
    };
}

#endif