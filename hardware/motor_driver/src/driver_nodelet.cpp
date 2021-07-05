#include <motor_driver/driver_nodelet.h>
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(motor_driver::MotorDriverNodelet, nodelet::Nodelet)

namespace motor_driver
{
    void MotorDriverNodelet::onInit()
    {
        NODELET_DEBUG("Initializing motor driver nodelet...");
        Setup(getNodeHandle());
    }

    void MotorDriverNodelet::Setup(ros::NodeHandle& nh)
    {
        try
        {
            nh_ = nh;
            driver_ = Driver(nh);
            motor_command_sub_ = nh.subscribe("movement_command", 1000, &MotorDriverNodelet::motor_command_callback, this);
            movement_state_pub_ = nh.advertise<MovementState>("movement_state", 5);
        }
        catch(...)
        {
            ROS_ERROR("Failed motor actuator node set up\n");
        }
    }
    
    void MotorDriverNodelet::motor_command_callback(const MotorCommand& cmd)
    {
        driver_.run(cmd, state_);
        movement_state_pub_.pub(state_);
    }
}