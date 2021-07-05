#include "motor_driver/stance_controller.h"

motor_driver::StanceController::StanceController(const double z_time_constant, const double dt)
    : z_time_constant_(z_time_constant), dt_(dt)
{}

geometry_msgs::Point motor_driver::StanceController::next_foot_location(const int leg_index, const MovementState state, const MotorCommand command)
{
    tf::Transform foot_location;
    geometry_msgs::Point translation;
    foot_location.setOrigin(tf::Vector3(state.foot_locations[leg_index].x, state.foot_locations[leg_index].y, state.foot_locations[leg_index].z));
    tf::Transform increment = position_delta(foot_location.getOrigin().z(), state.height, command, translation);
    foot_location = foot_location*increment;

    geometry_msgs::Point next_foot_location;
    next_foot_location.x = foot_location.getOrigin().x() + translation.x;
    next_foot_location.y = foot_location.getOrigin().y() + translation.y;
    next_foot_location.z = foot_location.getOrigin().z() + translation.z;

    return next_foot_location;
}

tf::Transform motor_driver::StanceController::position_delta(const double z, const double height, const MotorCommand command, geometry_msgs::Point& translation)
{
    // calculate position and translation steps
    double dz = 1.0 / z_time_constant_ * (height-z);
    double dx = -command.horizontal_velocity[0] *dt_;
    double dy = -command.horizontal_velocity[1] * dt_;
    translation.x = dx;
    translation.y = dy;
    translation.z = dz;

    double dR = -command.yaw_rate * dt_;

    tf::Transform increment;
    tf::Quaternion q;
    q.setRPY(0,0,dR);
    increment.setRotation(q);
    return increment;
}



