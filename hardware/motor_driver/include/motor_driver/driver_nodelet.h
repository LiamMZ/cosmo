#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <motor_driver/driver.h>
#include <motor_driver/MotorCommand.h>
#include <motor_driver/MovementState.h>
#include <mutex>


namespace motor_driver
{

    class MotorDriverNodelet : public nodelet::Nodelet
    {
        public:
            /**
             * Function to initialize nodelet, calls setup function
             */
            virtual void onInit();
            /**
             * Function to set up nodelet, advertises motor command service
             * 
             * @param[in] nh - nodehandle for nodelet
             */
            void Setup(ros::NodeHandle& nh);

        private:
            /**
             * Callback for motor command service
             * 
             * @param[in] command - motor command for movement
             */
            void motor_command_callback(const MotorCommand& cmd);



            // member variable for motor driver 
            Driver driver_;
            MovementState state_;

            // member variable for left motor command subscriber
            ros::Subscriber motor_command_sub_;
            // member variable for motor MovementState publisher
            ros::Publisher movement_state_pub_;
            // member variable for node handle
            ros::NodeHandle nh_;
    };

}