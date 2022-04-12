#ifndef TINYQUAD_PLANNER_LEG_KINEMATICS_
#define TINYQUAD_PLANNER_LEG_KINEMATICS_

#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "sensor_msgs/Joy.h"
#include "urdf/model.h"
#include "visualization_msgs/Marker.h"

namespace legKinematics_ns
{
    enum class Mode
    {
        cartesian_mode,
        joint_mode,
        bezier_mode
    };

    class LegKinematics
    {
    public:
        LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror);
        ~LegKinematics();
        void start();

    private:
        ros::NodeHandle p_nh_;
        ros::AsyncSpinner spinner;
        ros::Publisher joint_pub;
        ros::Publisher marker_pub;
        ros::Subscriber joy_sub;
        ros::Subscriber bezier_sub;

        const double LOOP_RATE {20.0f};

        KDL::JntArray joy_joints;
        KDL::Frame foot_contact_frame;

        std::string prefix;
        int x_mirror;
        int y_mirror;

        std::string urdf_param {"/robot_description"};
        double timeout {0.005f};
        double eps {1e-5f};
        const std::string chain_start {"base_link"};
        const std::string chain_end {prefix + "C_J"};
        TRAC_IK::TRAC_IK tracik_solver;

        std::vector<std::string> joint_name
        {   
            prefix + "S_J", 
            prefix + "U_J", 
            prefix + "L_J", 
            prefix + "C_J"
        };

        const double bias_x_nominal {-0.01f};
        const double bias_y_nominal {0.00f};
        const double bias_z_nominal {0.045f};

        const double bias_x {0.120024f + bias_x_nominal};
        const double bias_y {0.050443f + bias_y_nominal};
        const double bias_z {-0.163388f + bias_z_nominal};

        const double scale_x {0.1f};
        const double scale_y {0.1f};
        const double scale_z {0.05f};

        visualization_msgs::Marker target_marker;
        visualization_msgs::Marker target_save_marker;
        uint32_t shape {visualization_msgs::Marker::CUBE};

        tf2_ros::TransformBroadcaster tf_bc;
        geometry_msgs::TransformStamped transformStamped;
        tf2::Transform tf;
        tf2::Quaternion q;

        sensor_msgs::JointState joint_state;
        sensor_msgs::JointState curr_joint_state;
        sensor_msgs::JointState prev_joint_state;


        void loadParam();
        void initTf();
        void initMarker();
        void createLeg();
        void inverseKinematics();
        void forwardKinematics();
        void pathSegmentation();
        void joyCb(const sensor_msgs::Joy::ConstPtr& msg);
        void bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg);

        Mode leg_mode = Mode::cartesian_mode;

    };
} // ns legKinematics_ns

#endif