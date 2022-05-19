#ifndef _TINYQUAD_PLANNER_BODY_KINEMATICS_
#define _TINYQUAD_PLANNER_BODY_KINEMATICS_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "visualization_msgs/Marker.h"

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
            const double LOOP_RATE_ {10.0f};
            ros::Rate rate_;
            
            // for joy control
            const double scale_x_ {0.1};
            const double scale_y_ {0.1};
            const double scale_z_ {0.1};

            // for RPY controll (1 rad = 57 deg, 20 deg = 0.35 rad)
            const double scale_roll_  {0.35};
            const double scale_pitch_ {0.35};
            const double scale_yaw_   {0.35};

            geometry_msgs::PoseStamped bodyCentroid_;
            std::vector<double> joints_;

            ros::Publisher jointGroupCommandPub_;
            ros::Publisher bodyCentroidPub_;

            ros::Publisher targetBodyCentroidPub_;

            std::vector<ros::Publisher> jointCommandPub_;

            ros::Subscriber joySub_;

            void StartLegKinematics();
            visualization_msgs::Marker CreateMarker(std::string frame_id, std::string ns, int id, int type, double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w, double scale, float colour[4]);
            void PublishMarker(const visualization_msgs::Marker& marker);
            std::vector<geometry_msgs::Pose> CalculateBodyInverseKinematics(const geometry_msgs::PoseStamped& bodyCentroid);
            geometry_msgs::PoseStamped CalculateBodyForwardKinematics(const std::vector<geometry_msgs::Pose>& legs_pose);

            void JoyCb(const sensor_msgs::Joy::ConstPtr &msg);

            // helper function
            std::vector<double> ConvertQuaternionToEuler(const geometry_msgs::Quaternion& q);
            geometry_msgs::Quaternion ConvertEulerToQuaternion(const double& roll, const double& pitch, const double& yaw);

    };

} //bodykinematics_ns

#endif