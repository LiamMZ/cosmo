#include <motor_driver/driver.h>
#include <ros/console.h>

motor_driver::Driver::Driver(ros::NodeHandle nh) : nh_(nh), smoothed_yaw_(0.0)
{
    // get gait parameters
    std::vector< std::vector<bool> > contact_phases;
    unsigned int num_phases;
    std::vector<unsigned int> phase_ticks;
    unsigned int phase_length;
    if(!get_gait_params(contact_phases, num_phases, phase_ticks, phase_length))
    {
        throw false;
    }
    gait_controller_ = GaitController(phase_length, num_phases, phase_ticks, phase_length);

    // get stance parameters
    double z_time_constant;
    double dt;
    if(!get_stance_params(z_time_constant, dt))
    {
        throw false;
    }
    stance_controller_ = StanceController(z_time_constant, dt);

    //get swing parameters
    SwingConfig swing_config;
    if(!get_swing_params(swing_config))
    {
        throw false;
    }
    swing_config.dt = dt;
    swing_controller_ = SwingController(swing_config);
    default_stance_ = swing_config.default_stance;

    hop_transition_mapping_[cosmo::REST] = cosmo::HOP;
    hop_transition_mapping_[cosmo::HOP] = cosmo::FINISHHOP;
    hop_transition_mapping_[cosmo::FINISHHOP] = cosmo::REST;
    hop_transition_mapping_[cosmo::TROT] = cosmo::HOP;

    trot_transition_mapping_[cosmo::REST] = cosmo::TROT;
    trot_transition_mapping_[cosmo::TROT] = cosmo::REST;
    trot_transition_mapping_[cosmo::FINISHHOP] = cosmo::TROT;
    trot_transition_mapping_[cosmo::HOP] = cosmo::TROT;

    activate_transition_mapping_[cosmo::DEACTIVATED] = cosmo::REST;
    activate_transition_mapping_[cosmo::REST] = cosmo::DEACTIVATED;
} // END DRIVER CONSTRUCTOR

bool motor_driver::Driver::get_swing_params(SwingConfig& swing_config)
{
    if(!nh_.get_param("/motor_driver/alpha" swing_config.alpha))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load alpha.");
        return false;
    }
    if(!nh_.get_param("/motor_driver/stance_ticks" swing_config.stance_ticks))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load stance_ticks.");
        return false;
    }
    if(!nh_.get_param("/motor_driver/beta" swing_config.beta))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load beta.");
        return false;
    }
    if(!nh_.get_param("/motor_driver/z_clearance" swing_config.z_clearance))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load alpha.");
        return false;
    }
    std::vector<double> foot_loc;
    geometry_msgs::Point point;
    for(unsigned int leg_index = 0; leg_index<4; leg_index++)
    {
         if(!nh_.get_param("/motor_driver/default_stance_"+leg_index, foot_loc))
        {
            ROS_ERROR_STREAM("motor_driver::Driver::get_swing_params Failed to load default_stance_"<< leg_index);
            return false;
        }
        point.x = foot_loc[0];
        point.y = foot_loc[1];
        point.z = foot_loc[2];
        default_stance.push_back(point);
    }
    return true;
}

bool motor_driver::Driver::get_stance_params(double& z_time_constant, double& dt)
{
    if(!nh_.get_param("/motor_driver/z_time_constant", z_time_constant))
    {
        ROS_ERROR("motor_driver::Driver::get_stance_params Failed to load z_time_constant.");
        return false;
    }
    if(!nh_.get_param("/motor_driver/dt", dt))
    {
        ROS_ERROR("motor_driver::Driver::get_stance_params Failed to load dt.")
        return false;
    }
    return true;
}

bool motor_driver::Driver::get_gait_params(const std::vector< std::vector<bool> >& contact_phases, unsigned int& num_phases, std::vector<unsigned int>& phase_tics,  unsigned int& phase_length)
{
    // get contact phases
    std::vector<bool> phase;
    for(unsigned int leg_index = 0; leg_index<4; leg_index++)
    {
        if(!nh_.get_param("/motor_driver/contact_phases_"+leg_index, phase))
        {
            ROS_ERROR_STREAM("motor_driver::Driver::get_gait_params Failed to load contact_phase_"<< leg_index);
            return false;
        }
        contact_phases.push_back(phase);
    }

    if(!nh_.get_param("/motor_driver/num_phases", num_phases))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load num_phases.");
        return false;
    }

    if(!nh_.get_param("/motor_driver/phase_ticks", phase_ticks))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load phase_ticks.");
        return false;
    }

    if(!nh_.get_param("/motor_driver/phase_length", phase_length))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load phase_length.");
        return false;
    }
    return true
}
std::vector<geometry_msgs::Point> motor_driver::Driver::step_gait(const MotorCommand command, std::vector<bool>& contact_modes)
{

}

std::vector< std::vector<double> > motor_driver::Driver::run(const MotorCommand command)
{

}

std::vector< std::vector<double> > set_pose_to_default()
{

}

