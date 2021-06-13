#include "motor_driver/kinematics.h"


motor_driver::Kinematics::Kinematics(const double abduction_offset, const double l1, const double l2, const std::vector<geometry_msgs::Point> leg_origins)
 : body_offset(leg_origins), LEG_L1(l1), LEG_L2(l2), ABDUCTION_OFFSET(abduction_offset)
{}

std::vector<double> motor_driver::Kinematics::leg_explicit_inverse_kinematics(const geometry_msgs::Point r_body_foot, const unsigned int leg_index)
{
    double x = r_body_foot.x;
    double y = r_body_foot.y;
    double z = r_body_foot.z;

    // For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
    double abduction_offset = leg_index==1 || leg_index==3 ? ABDUCTION_OFFSET : -1*ABDUCTION_OFFSET;

    // Distance from the body to the foot in the y-z axes in the body frame 
    double R_body_foot_yz = pow(y*y + z*z, 0.5);
    // Distance from ad/abduction axis to foot in the body frame
    double R_hip_foot_yz = pow((R_body_foot_yz*R_body_foot_yz) - (abduction_offset*abduction_offset), 0.5);

    // Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
    double arc_cos_arg = abduction_offset/R_body_foot_yz;
    arc_cos_arg = clamp(arc_cos_arg, -0.99, 0.99);
    double phi = acos(arc_cos_arg);

    // Angle of the y-z projection of the hip-to-foot vector
    double hip_foot_angle = atan2(z, y);

    // ad/abduction angle relative to the positive y axis
    double abduction_angle = phi + hip_foot_angle;

    // angle between tilted negative z axis and hip-to-foot vector
    double theta = atan2(-x,R_hip_foot_yz);

    //distance between hip and foot
    double R_hip_foot = pow(x*x + R_hip_foot_yz*R_hip_foot_yz, 0.5);
    
    // angle between vector going from hip to foot and leg link L1
    arc_cos_arg = (LEG_L1*LEG_L1 + R_hip_foot*R_hip_foot - LEG_L2*LEG_L2)/
                        (2*R_hip_foot*LEG_L1);
    arc_cos_arg = clamp(arc_cos_arg, -0.99, 0.99);
    double trident = acos(arc_cos_arg);
    
    // angle of first link relative to tilted negative z axis
    double hip_angle = trident+theta;

    // angle between links l1 and l2
    arc_cos_arg = (LEG_L1*LEG_L1 + LEG_L2*LEG_L2 - R_hip_foot*R_hip_foot)/
                    (2*LEG_L1*LEG_L2);
    arc_cos_arg = clamp(arc_cos_arg, -0.99, 0.99);
    double beta = acos(arc_cos_arg);

    // angle of second link relative to tilted negative z axis
    double knee_angle = hip_angle - (acos(0.0)-beta);

    std::vector<double> ret{abduction_angle, hip_angle, knee_angle};
    return ret;
}

std::vector< std::vector<double> > motor_driver::Kinematics::four_legs_inverse_kinematics(std::vector<geometry_msgs::Point> leg_poses)
{
    std::vector< std::vector<double> > alpha;
    geometry_msgs::Point body_offset;
    unsigned int leg_index = 0;
    for(geometry_msgs::Point pose : leg_poses)
    {
        pose.x = pose.x-body_offset.x;
        pose.y = pose.y-body_offset.y;
        pose.z = pose.z-body_offset.z;
        alpha.push_back(leg_explicit_inverse_kinematics(pose, leg_index));
        leg_index++;
    }
    return alpha;
}

double motor_driver::Kinematics::clamp(double value, const double low, const double high)
{
    if (value < low) return low;
    else if (value>high) return high;
    return value;
}