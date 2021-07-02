#include <motor_driver/driver.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>


motor_driver::Driver::Driver(ros::NodeHandle nh) : nh_(nh), smoothed_yaw_(0.0)
{
    bool isLoadSuccesful = update_parameters();
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

bool motor_driver::Driver::update_parameters()
{
    // get gait parameters
    std::vector< std::vector<bool> > contact_phases;
    unsigned int num_phases = 0;
    std::vector<unsigned int> phase_ticks;
    unsigned int phase_length = 0;
    if(!get_gait_params(contact_phases, num_phases, phase_ticks, phase_length))
    {
        return false;
    }
    gait_controller_ = GaitController(phase_length, num_phases, phase_ticks, phase_length);

    // get stance parameters
    double z_time_constant = 0.0;
    double dt = 0;
    if(!get_stance_params(z_time_constant, dt))
    {
        return false;
    }
    stance_controller_ = StanceController(z_time_constant, dt);

    //get swing parameters
    SwingConfig swing_config;
    if(!get_swing_params(swing_config))
    {
        return false;
    }
    swing_config.dt = dt;
    swing_controller_ = SwingController(swing_config);
    default_stance_ = swing_config.default_stance;

    //get kinematics parameters
    double abduction_offset=0.0, L1=0.0, L2=0.0;
    std::vector< geometry_msgs::Point> leg_origins;
    if(!get_kinematics_params(abduction_offset, L1, L2, leg_origins))
    {
        return false;
    }
    inverse_kinematics_ = Kinematics(abduction_offset, L1, L2, leg_origins);

    return true;
}

bool motor_driver::Driver::get_swing_params(SwingConfig& swing_config)
{
    if(!nh_.getParam("/motor_driver/alpha", swing_config.alpha))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load alpha.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/stance_ticks", swing_config.stance_ticks))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load stance_ticks.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/beta", swing_config.beta))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load beta.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/z_clearance", swing_config.z_clearance))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load z_clearance.");
        return false;
    }
    std::vector<double> foot_loc;
    geometry_msgs::Point point;
    for(unsigned int leg_index = 0; leg_index<4; leg_index++)
    {
         if(!nh_.getParam("/motor_driver/default_stance_"+leg_index, foot_loc))
        {
            ROS_ERROR_STREAM("motor_driver::Driver::get_swing_params Failed to load default_stance_"<< leg_index);
            return false;
        }
        point.x = foot_loc[0];
        point.y = foot_loc[1];
        point.z = foot_loc[2];
        default_stance.push_back(point);
    }

    if(!nh_.getParam("/motor_driver/swing_ticks", swing_ticks_))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load swing_ticks.");
        return false;
    }
    return true;
}

bool motor_driver::Driver::get_stance_params(double& z_time_constant, double& dt)
{
    if(!nh_.getParam("/motor_driver/z_time_constant", z_time_constant))
    {
        ROS_ERROR("motor_driver::Driver::get_stance_params Failed to load z_time_constant.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/dt", dt))
    {
        ROS_ERROR("motor_driver::Driver::get_stance_params Failed to load dt.")
        return false;
    }
    if(!nh_.getParam("/motor_driver/default_z_ref_", default_z_ref_))
    {
        ROS_ERROR("motor_driver::Driver::get_stance_params Failed to load default_z_ref_.")
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
        if(!nh_.getParam("/motor_driver/contact_phases_"+leg_index, phase))
        {
            ROS_ERROR_STREAM("motor_driver::Driver::get_gait_params Failed to load contact_phase_"<< leg_index);
            return false;
        }
        contact_phases.push_back(phase);
    }

    if(!nh_.getParam("/motor_driver/num_phases", num_phases))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load num_phases.");
        return false;
    }

    if(!nh_.getParam("/motor_driver/phase_ticks", phase_ticks))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load phase_ticks.");
        return false;
    }

    if(!nh_.getParam("/motor_driver/phase_length", phase_length))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load phase_length.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/max_yaw_rate", max_yaw_rate_))
    {
        ROS_ERROR("motor_driver::Driver::get_gait_params Failed to Load max_yaw_rate.");
        return false;
    }
    return true
}

bool motor_driver::Driver::get_kinematics_params(double& abduction_offset, double& L1, double& L2, std::vector<geometry_msgs::Point>& leg_origins)
{
    if(!nh_.getParam("/motor_driver/L1", L1))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load L1.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/L2", L2))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load L2.");
        return false;
    }
    if(!nh_.getParam("/motor_driver/abduction_offset", abduction_offset))
    {
        ROS_ERROR("motor_driver::Driver::get_swing_params Failed to load abduction_offset.");
        return false;
    }
    std::vector<double> foot_loc;
    geometry_msgs::Point point;
    for(unsigned int leg_index = 0; leg_index<4; leg_index++)
    {
         if(!nh_.getParam("/motor_driver/leg_origin_"+leg_index, foot_loc))
        {
            ROS_ERROR_STREAM("motor_driver::Driver::get_swing_params Failed to load leg_origin_"<< leg_index);
            return false;
        }
        point.x = foot_loc[0];
        point.y = foot_loc[1];
        point.z = foot_loc[2];
        leg_origins.push_back(point);
    }
    return true;
}

std::vector<geometry_msgs::Point> motor_driver::Driver::step_gait(const MotorCommand command, std::vector<bool>& contact_modes, cosmo::State& state)
{
    contact_modes = gait_controller_.contacts(state.ticks);
    std::vector< geometry_msgs::Point> new_foot_locations, foot_location;
    geometry_msgs::Point new_location;
    bool contact_mode;
    double swing_proportion;
    for( unsigned int leg_index = 0; leg_index<4; leg_index++)
    {
        contact_mode = contact_modes[leg_index];
        foot_location = state.foot_locations[leg_index];
        if(contact_mode)
        {
            new_location = stance_controller_.next_foot_location(leg_index, state, command);
        }
        else
        {
            swing_proportion = gait_controller_.subphase_ticks(state.ticks)/swing_ticks_;
            new_location = swing_controller_.next_foot_location(swing_proportion, leg_index, state, command);
        }
        new_foot_locations[leg_index] = new_location;
    }
    return new_foot_locations;
}

std::vector< std::vector<double> > motor_driver::Driver::run(const MotorCommand command, cosmo::State& state)
{
    if(command.activate_event)
    {
        state.behavior_state = activate_transition_mapping_[state.behavior_state];
    }
    else if (command.trot_event)
    {
        state.behavior_state = trot_transition_mapping_[state.behavior_state];
    }
    else if (command.hop_event)
    {
        state.behavior_state = hop_transition_mapping_[state.behavior_state];
    }
    std::vector< bool> contact_modes;
    std::vector< geometry_msgs::Point> rotated_foot_locations; 
    double roll, pitch, yaw;
    switch(state.behavior_state)
    {
        case cosmo::TROT :
            state.foot_locations = step_gait(command, contact_modes, state);
            rotated_foot_locations = rotate_foot_locations(command.roll, command.pitch, command.yaw, state.foot_locations);
            roll = state.roll;
            yaw = state.yaw;
            roll = CORRECTION_FACTOR * Util::clamp(state.roll, -MAX_TILT, MAX_TILT);
            pitch = CORRECTION_FACTOR * Util::clamp(state.pitch, -MAX_TILT, MAX_TILT);
            rotated_foot_locations = rotate_foot_locations(roll, pitch, yaw, rotated_foot_locations);
            state.joint_angles = inverse_kinematics_.four_legs_inverse_kinematics(rotated_foot_locations);
            break;
        case cosmo::HOP :
            for(int i = 0; i<4; i++)
            {
                state.foot_locations[i] = default_stance[i];
                state.foot_locations[i].z -= 0.09;
            }
            state.joint_angles = inverse_kinematics_.four_legs_inverse_kinematics(state.foot_locations);
            break;
        case cosmo::FINISHHOP :
            for(int i = 0; i<4; i++)
            {
                state.foot_locations[i] = default_stance[i];
                state.foot_locations[i].z -= 0.22;
            }
            state.joint_angles = inverse_kinematics_.four_legs_inverse_kinematics(state.foot_locations);
            break;
        case cosmo::REST:
            double yaw_proportion = command.yaw_Rate / max_yaw_rate_;
            smoothed_yaw_ += dt_ * Util::clipped_first_order_filter( smoothed_yaw_,
                                                                   yaw_proportion * -max_stance_yaw_,
                                                                   max_stance_yaw_rate_,
                                                                   yaw_time_constant_);
            for(int i = 0; i<4; i++)
            {
                state.foot_locations[i] = default_stance[i];
                state.foot_locations[i].z += command.height;
            }
            std::vector<geometry_msgs::Point> rotated_foot_locations = rotate_foot_locations(command.roll, command.pitch, smoothed_yaw_);
            state.joint_angles = inverse_kinematics_.four_foot_inverse_kinematics(rotated_foot_locations);
            break;
        state.ticks +=1;
        state.pitch = command.pitch;
        state.roll = command.roll;
        state.height = command.height;
    }
}

std::vector< geometry_msgs::Point> motor_driver::Driver::rotate_foot_locations(const double roll, const double pitch, const double yaw, const std::vector< geometry_msgs::Point>& foot_locations)
{
    std::vector<geometry_msgs::Point> rotated_leg_poses;
    geometry_mst::Point new_pose;

    // set up tf for rotation
    tf::Transform update;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    update.setRotation(q);
    // rotate points
    for(int leg = 0; leg<4; leg++)
    {
        tf::Transform leg_tf;
        leg_tf.setOrigin(tf::Vector3(foot_locations[leg].x, foot_locations[leg].y, foot_locations[leg].z))
        leg_tf = update*leg_tf;
        new_pose.x = leg_tf.getOrigin().x();
        new_pose.y = leg_tf.getOrigin().y();
        new_pose.z = leg_tf.getOrigin().z();
        rotated_leg_poses.push_back(new_pose);
    }
    return rotated_leg_poses;
}

std::vector< std::vector<double> > set_pose_to_default(cosmo::State& state)
{
    for(int i = 0; i<4; i++)
    {
        state.foot_locations[i] = default_stance[i];
        state.foot_locations[i].z += default_z_ref_;
    }
    state.joint_angles = inverse_kinematics_.four_leg_inverse_kinematics(state.foot_locations);
}

