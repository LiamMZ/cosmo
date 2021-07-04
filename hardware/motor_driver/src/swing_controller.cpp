#include <motor_driver/swing_controller.h>
#include <tf/transform_broadcaster.h>
/*
* Sets swing config var
*/
motor_driver::SwingController::SwingController(motor_driver::SwingConfig config) : config_(config)
{
}

/*
* Function to calculate next foot location in gait
*/
geometry_msgs::Point motor_driver::SwingController::next_foot_location(const double swing_phase, const int leg_index, State state, const MotorCommand command, bool triangular)
{
    if(swing_phase>1 || swing_phase<0)
    {
        throw false;
    }
    geometry_msgs::Point foot_location = state.foot_locations[leg_index];
    double swing_height_  = swing_height(swing_phase);
    geometry_msgs::Point touchdown_location = raibert_touchdown_location(leg_index, command);
    double time_left = config_.dt * config_.dt * (1.0-swing_phase);

    geometry_msgs::Point v;
    v.x = (touchdown_location.x-foot_location.x)/(time_left);
    v.y = (touchdown_location.y-foot_location.y)/(time_left);
    v.z = 0;

    geometry_msgs::Point next_foot_location;
    next_foot_location.x = foot_location.x + (v.x*config_.dt);
    next_foot_location.z = foot_location.y + (v.y*config_.dt);
    next_foot_location.z = swing_height_ + command.height;

    return next_foot_location;
}

double motor_driver::SwingController::swing_height(const double swing_phase, bool triangular)
{
    double swing_height_ = 0.0;
    if(triangular)
    {
        if (swing_phase<0.5)
        {
            swing_height_ = swing_phase / 0.5 * config_.z_clearance;
        }
        else
        {
            swing_height_ = config_.z_clearance * (1 - (swing_phase - 0.5) / 0.5);
        }
    }
    return swing_height_;
}

geometry_msgs::Point motor_driver::SwingController::raibert_touchdown_location(const int leg_index, const MotorCommand command)
{
    double theta = config_.beta * config_.stance_ticks * config_.dt * command.yaw_rate;
    tf::Transform update;
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    update.setRotation(q);

    tf::Transform default_stance;
    default_stance.setOrigin(tf::Vector3(config_.default_stance[leg_index].x,
                                        config_.default_stance[leg_index].y,
                                        config_.default_stance[leg_index].z));
    update = default_stance*update;

    geometry_msgs::Point touchdown_location;
    touchdown_location.x = update.getOrigin().x() 
                           * config_.alpha * config_.stance_ticks 
                           * config_.dt * command.horizontal_velocity[0];
    
    touchdown_location.y = update.getOrigin().y() 
                           * config_.alpha * config_.stance_ticks 
                           * config_.dt * command.horizontal_velocity[1];
    
    touchdown_location.z = update.getOrigin().z();
    return touchdown_location;
}
