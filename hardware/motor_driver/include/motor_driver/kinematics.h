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
        /*
        * Constructor initializes private configuration variables
        * @param[in] abduction_offset - offset of ad/abduction axes in y from COG
        * @param[in] l1 - distance to link l1 on robot leg in meters
        * @param[in] l2 - distance to link l2 on robot leg in meters
        * @param[in] leg_origins - points of leg origins in the body frame
        *
        */
        Kinematics(const double abduction_offset, const double l1, const double l2, const std::vector<geometry_msgs::Point> leg_origins);

        /*
        * Function to perform inverse kinematics for each of four passed in foot positions
        * @param[in] foot_poses - desired foot positions for each of the four legs
        * @returns - vector of twelve angles corresponding to the three motors for each of the four legs
        */
        std::vector< std::vector<double> > four_legs_inverse_kinematics(std::vector<geometry_msgs::Point> foot_poses);

    private:
        /*
        * Function to calculate inverse kinematics on a single foot position
        * @param[in] r_body_foot - desired position of the foot
        * @param[in] leg_index - index for given leg
        */
        std::vector<double> leg_explicit_inverse_kinematics(const geometry_msgs::Point r_body_foot, const unsigned int leg_index);
        

        // distance to link l1 for each of the four legs
        double LEG_L1;
        // distance to link l2 for each of the four legs
        double LEG_L2;
        // offset from origin to 
        double ABDUCTION_OFFSET;
        // offset of ad/abduction axes in y from COG
        std::vector<geometry_msgs::Point> body_offset;

    };
}//end namespace motor driver