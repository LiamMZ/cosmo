#ifndef STATE_H
#define STATE_H

#include <vector>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Point.h"

namespace cosmo
{
    enum BehaviorState
    {
        DEACTIVATED = -1,
        REST,
        TROT,
        HOP,
        FINISHHOP
    };

    struct State
    {
        State() : horizontal_velocity({0.0, 0.0}), yaw_rate(0.0), height(-0.16), pitch(0.0), roll(0.0), activation(0), behavior_state(REST), ticks(0){};
        std::vector<double> horizontal_velocity;
        double yaw_rate;
        double height;
        double pitch;
        double roll;
        int activation;
        BehaviorState behavior_state;

        unsigned long ticks;
        std::vector<geometry_msgs::Point> foot_locations;
        std::vector< std::vector<double> > joint_angles;
    };

}
#endif