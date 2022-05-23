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

    // struct bodyRectangle{
    //     double length;
    //     double width;
    // };
    
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
            const double SCALE_X_ {0.2};
            const double SCALE_Y_ {0.2};
            const double SCALE_Z_ {0.2};

            // for RPY controll (1 rad = 57 deg, 20 deg = 0.35 rad)
            const double SCALE_ROLL_  {1.0};
            const double SCALE_PITCH_ {1.0};
            const double SCALE_YAW_   {1.0};

            geometry_msgs::Pose defaultPose_;
            std_msgs::ColorRGBA defaultColour_;

            bool usePoints_;
            std::vector<geometry_msgs::Point> bodyCorners_;
            std::vector<geometry_msgs::Point> bodyCornersInitial_;
            // bodyRectangle bodyRec_;
            double bodyLength_;
            double bodyWidth_;
            geometry_msgs::PoseStamped bodyCentroid_;
            std::vector<double> joints_;

            ros::Publisher jointGroupCommandPub_;
            ros::Publisher bodyCentroidPub_;
            ros::Publisher targetBodyCentroidPub_;
            ros::Publisher markerPub_;

            ros::Subscriber joySub_;

            std::vector<ros::Publisher> jointCommandPub_;

            geometry_msgs::Pose markerPose_;
            geometry_msgs::Pose markerPosePrevious_;

            // kinematics function
            void StartLegKinematics();
            std::vector<geometry_msgs::Point> CalculateBodyInverseKinematics(const geometry_msgs::Pose& center_target, std::vector<geometry_msgs::Point>& points);
            geometry_msgs::PoseStamped CalculateBodyForwardKinematics(const std::vector<geometry_msgs::Pose>& legs_pose);

            // math helper function
            std::vector<double> ConvertQuaternionToEuler(const geometry_msgs::Quaternion& q);
            geometry_msgs::Quaternion ConvertEulerToQuaternion(const double roll, const double pitch, const double yaw);
            std::vector<geometry_msgs::Point> FindPointsFromSize(std::vector<double>& size);
            std::vector<double> FindSizeFromPoints(std::vector<geometry_msgs::Point>& points);

            // visualise function
            visualization_msgs::Marker CreateMarker(std::string frame_id, std::string ns, int id, int type, geometry_msgs::Pose& pose, double scale, std_msgs::ColorRGBA& colour);
            void PublishMarker(const visualization_msgs::Marker& marker);
            void PublishMarker(const std::vector<visualization_msgs::Marker>& markers);
            void UpdateMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose pose);
            std::vector<visualization_msgs::Marker> DrawRecangle(std::vector<geometry_msgs::Point>& points);
            // void DrawRecangle(const double length, const double width);

            // ROS function
            void LoadParam();
            void DefineDefaultVariables();
            void JoyCb(const sensor_msgs::Joy::ConstPtr &msg);
        };

} //bodykinematics_ns

#endif