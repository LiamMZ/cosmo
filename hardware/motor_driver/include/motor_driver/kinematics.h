// Cosmo
#include <cosmo/mobility_config.h>

//Ros packages
#include <geometry_msgs/Point.h>

// Std lib
#include <math.h>
#include <vector>

#pragma once
namespace motor_driver
{
class Kinematics
{
public:
    Kinematics(cosmo::MobilityConfig config);
    void four_legs_inverse_kinematics();
private:
    std::vector<double> leg_explicit_inverse_kinematics(geometry_msgs::Point r_body_foot, unsigned int leg_index);
    cosmo::MobilityConfig config_;
};

}//end namespace motor driver