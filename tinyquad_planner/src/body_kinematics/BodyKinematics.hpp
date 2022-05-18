#ifndef _TINYQUAD_PLANNER_BODY_KINEMATICS_
#define _TINYQUAD_PLANNER_BODY_KINEMATICS_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

#include <iostream>
#include <vector>
#include <string>

#include "../legKinematics/legKinematics.hpp"

namespace bodykinematics_ns
{
    enum class BodyState{
        WALKING,
        TOPPLED,
        RESTING,
        SQUATING
    };
    class BodyKinematics
    {
        public:
            BodyKinematics(ros::NodeHandle &nh, const std::string name);
            ~BodyKinematics();
            void Start();

        private:
            ros::NodeHandle p_nh_;
            const double LOOP_RATE {10.0f};
            ros::Rate rate;

            geometry_msgs::PoseStamped bodyCentroid_;
            std::vector<double> joints_;

            ros::Publisher jointGroupCommandPub;
            ros::Publisher bodyCentroidPub;

            std::vector<ros::Publisher> jointCommandPub;

            ros::Subscriber joySub;

            void startLegKinematics();
            std::vector<double> CalculateBodyInverseKinematics(const geometry_msgs::PoseStamped::ConstPtr &bodyCentroid);
            geometry_msgs::PoseStamped CalculateBodyForwardKinematics(const std::vector<double> joints);

            void JoyCb(const sensor_msgs::Joy::ConstPtr &msg);

    };

} //bodykinematics_ns

#endif