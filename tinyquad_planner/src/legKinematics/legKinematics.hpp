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

    class Colour
    {
        public:
            float RED[4] {1.0f, 0.0f, 0.0f, 1.0f};
            float GREEN[4] {0.0f, 1.0f, 0.0f, 1.0f};
            float BLUE[4] {0.0f, 0.0f, 1.0f, 1.0f};
    };

    class LegKinematics
    {
    public:
        LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror);
        ~LegKinematics();
        void start();

    private:
        ros::NodeHandle p_nh_;
        ros::Publisher joint_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber bezier_sub_;
        ros::Subscriber joy_sub_;
        ros::Subscriber imu_sub_;
        std::vector<ros::Publisher> legPubs_;

        ros::Publisher LBU_J_pub_;
        ros::Publisher LBL_J_pub_;
        ros::Publisher RBU_J_pub_;
        ros::Publisher RBL_J_pub_;
        
        ros::Publisher LFU_J_pub_;
        ros::Publisher LFL_J_pub_;
        ros::Publisher RFU_J_pub_;
        ros::Publisher RFL_J_pub_;

        const double LOOP_RATE_ {100.0};

        KDL::JntArray joy_joints_;
        KDL::Frame foot_contact_frame_;

        std::string prefix_;
        int x_mirror_;
        int y_mirror_;

        std::string urdf_param_ {"/robot_description"};
        double timeout_ {0.005};
        double eps_ {1e-5};
        const std::string chain_start_ {"base_link"};
        const std::string chain_end_ {prefix_ + "C2_L"};
        TRAC_IK::TRAC_IK tracik_solver_;

        std::vector<std::string> joint_name_
        {   
            prefix_ + "S_J", 
            prefix_ + "U_J", 
            prefix_ + "L_J", 
            prefix_ + "C_J",
            prefix_ + "C2_J"
        };

        const double bias_x_nominal_ {0.02};
        const double bias_y_nominal_ {0.00};
        const double bias_z_nominal_ {0.045};

        const double bias_x_ {x_mirror_ * (0.120024 + bias_x_nominal_)};
        const double bias_y_ {y_mirror_ * (0.050443 + bias_y_nominal_)};
        const double bias_z_ {-0.163388 + bias_z_nominal_};

        const double scale_x_ {0.1};
        const double scale_y_ {0.1};
        const double scale_z_ {0.05};

        visualization_msgs::Marker target_marker_;

        tf2_ros::TransformBroadcaster tf_bc_;
        geometry_msgs::TransformStamped transformStamped_;
        tf2::Transform tf_;
        tf2::Quaternion q_;

        sensor_msgs::JointState joint_state_;
        sensor_msgs::JointState curr_joint_state_;
        sensor_msgs::JointState prev_joint_state_;

        KDL::Chain chain_;
        KDL::JntArray lower_limit_, upper_limit_;

        uint number_of_joints_ {5};

        KDL::JntArray curr_joint_array_;

        KDL::JntArray ik_result_;
        KDL::ChainFkSolverPos_recursive fk_solver_;

        const std::string robot_namespace_ {"tinyquad/"};

        geometry_msgs::Twist cmd_vel_msg_;

        Colour marker_colour_;

        void loadParam();
        void initLegPubs();
        void initTf(std::string frame_id, std::string child_frame_id);
        void initMarker(std::string frame_id, std::string ns, int id, int type, double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w, double scale, float colour[4]);
        void createLeg();
        bool checkKDLChain();
        std::vector<KDL::JntArray>  getKDLLimits();
        KDL::JntArray getJointsNominal(const KDL::JntArray& lower_limit, const KDL::JntArray& upper_limit);
        KDL::JntArray inverseKinematics(KDL::JntArray current_joint, KDL::Frame target_frame);
        // void initForwardKinematics();
        double getNumberOfJoints(KDL::Chain& chain);
        void forwardKinematics();
        // sensor_msgs::JointState convertJntArraytoJointState(const KDL::JntArray& jntarray);

        void publishJointStates(const KDL::JntArray& jntarray);

        void updateFSM();
        // void pathSegmentation();
        void joyCb(const sensor_msgs::Joy::ConstPtr& msg);
        void bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg);
        void imuCb(const sensor_msgs::ImuConstPtr& msg);

        Mode leg_mode_ {Mode::cartesian_mode};
        // Mode leg_mode_ {Mode::wheel_mode};

    };
} // ns legKinematics_ns

#endif