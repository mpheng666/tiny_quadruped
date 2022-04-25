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

#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

namespace legKinematics_ns
{
    enum class Mode
    {
        cartesian_mode,
        joint_mode,
        bezier_mode,
        wheel_mode
    };

    class LegKinematics
    {
    public:
        LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror);
        ~LegKinematics();
        void start();

    private:
        ros::NodeHandle p_nh_;
        // ros::AsyncSpinner spinner;
        ros::Publisher joint_pub;
        ros::Publisher marker_pub;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber bezier_sub;
        ros::Subscriber joy_sub;
        ros::Subscriber imu_sub;
        std::vector<ros::Publisher> legPubs;

        ros::Publisher LBU_J_pub;
        ros::Publisher LBL_J_pub;
        ros::Publisher RBU_J_pub;
        ros::Publisher RBL_J_pub;
        
        ros::Publisher LFU_J_pub;
        ros::Publisher LFL_J_pub;
        ros::Publisher RFU_J_pub;
        ros::Publisher RFL_J_pub;

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
        const std::string chain_end {prefix + "C_L"};
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

        const double bias_x {x_mirror * (0.120024f + bias_x_nominal)};
        const double bias_y {y_mirror * (0.050443f + bias_y_nominal)};
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

        KDL::Chain chain;
        KDL::JntArray lower_limit, upper_limit;

        uint number_of_joints {4};

        KDL::JntArray curr_joint_array;

        KDL::JntArray ik_result;
        KDL::ChainFkSolverPos_recursive fk_solver;

        const std::string robot_namespace {"tinyquad/"};

        geometry_msgs::Twist cmd_vel_msg;

        void loadParam();
        void initLegPubs();
        void initTf();
        void initMarker();
        void createLeg();
        bool checkKDLChain();
        void getKDLLimits();
        void getJointsNominal();
        void inverseKinematics();
        void initForwardKinematics();
        void forwardKinematics();
        void updateFSM();
        // void pathSegmentation();
        void joyCb(const sensor_msgs::Joy::ConstPtr& msg);
        void bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg);
        void imuCb(const sensor_msgs::ImuConstPtr& msg);

        // Mode leg_mode {Mode::cartesian_mode};
        Mode leg_mode {Mode::wheel_mode};

    };
} // ns legKinematics_ns

#endif