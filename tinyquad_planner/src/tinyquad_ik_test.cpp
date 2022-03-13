#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "sensor_msgs/Joy.h"
#include "urdf/model.h"

const double PI = 3.14159;
KDL::JntArray joy_joints(4);
KDL::Frame leg_contact_frame;

enum class Mode
{
    cartesian_mode,
    joint_mode
};

Mode leg_mode = Mode::cartesian_mode;

void joyCb(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->buttons[0] == 1)
    {
        leg_mode = Mode::cartesian_mode;
        leg_contact_frame.p.x(msg->axes[0]+0.120024);
        leg_contact_frame.p.y(msg->axes[1]+0.050443);
        leg_contact_frame.p.z(msg->axes[2]-0.163388);
    }
    else if (msg->buttons[1] == 1)
    {
        leg_mode = Mode::joint_mode;
        joy_joints(0) = msg->axes[0];
        joy_joints(1) = msg->axes[1];
        joy_joints(2) = msg->axes[2];
        joy_joints(3) = msg->axes[3];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tq_ik_test_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCb);
    ros::Rate loop_rate(10);

    // std::vector<std::string> joint_name = {"LFS_J", "LFU_J", "LFL_J",
    //                                        "RFS_J", "RFU_J", "RFL_J",
    //                                        "LBS_J", "LBU_J", "LBL_J",
    //                                        "RBS_J", "RBU_J", "RBL_J"};
    std::vector<std::string> joint_name = {"LFS_J", "LFU_J", "LFL_J", "LFC_J"};
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState curr_joint_state;
    sensor_msgs::JointState prev_joint_state;

    // init inverse kinematics solver
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start = "base_link";
    // std::string chain_start = "LFS_L";
    std::string chain_end = "LFC_L";
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    // get KDL chain
    KDL::Chain chain;
    bool valid_chain = tracik_solver.getKDLChain(chain);
    if (!valid_chain)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }

    // get joint lower limits, joint upper limits
    KDL::JntArray lower_limit, upper_limit;
    bool valid_limits = tracik_solver.getKDLLimits(lower_limit, upper_limit);
    if (!valid_limits)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }

    // init forward kinematics solver
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    uint number_of_joints = chain.getNrOfJoints();
    ROS_INFO("joints number: %d", number_of_joints);

    // calculate nominal joint position
    KDL::JntArray nominal(number_of_joints);
    for (size_t i = 0; i < nominal.data.size(); i++)
    {
        nominal(i) = (lower_limit(i) + upper_limit(i)) / 2.0;
        printf("nominal %li: %f \n", i, nominal(i));
    }
    KDL::JntArray ik_result;
    KDL::JntArray curr_joint_array = nominal;

    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };

    while (ros::ok())
    {
        switch (leg_mode)
        {
            // 1. Cartesian mode: listen joy::end_cartesian, calculate joints from inverse kinematics
            case Mode::cartesian_mode:
                {
                    joint_state.header.stamp = ros::Time::now();
                    int rc = tracik_solver.CartToJnt(curr_joint_array, leg_contact_frame, ik_result);
                    joint_state.name.resize(number_of_joints);
                    joint_state.position.resize(number_of_joints);
                    for (size_t i = 0; i < number_of_joints; i++)
                    {
                        ROS_INFO("ik_result: %f", double(ik_result(i)));
                        joint_state.name[i] = joint_name[i];
                        joint_state.position[i] = ik_result(i);
                        curr_joint_array.data[i] = ik_result(i);
                    }
                    print_frame_lambda(leg_contact_frame);
                    ROS_INFO("Leg is in cartesian mode");
                    break;
                }

            // 2. Joint mode: listen to joy::joints, calculate end_cartesian from forward kinematics
            case Mode::joint_mode:
                {
                    joint_state.header.stamp = ros::Time::now();
                    fk_solver.JntToCart(joy_joints, leg_contact_frame);
                    joint_state.name.resize(number_of_joints);
                    joint_state.position.resize(number_of_joints);
                    for (size_t i = 0; i < number_of_joints; i++)
                    {
                        joint_state.name[i] = joint_name[i];
                        joint_state.position[i] = joy_joints(i);
                    }
                    print_frame_lambda(leg_contact_frame);
                    ROS_INFO("Leg is in joint mode");
                    break;
                }

            // 3. Rest mode: return all joints to nominal joints
            default:
                std::cout << "default mode \n";
                break;
                // {
                //     joint_state.header.stamp = ros::Time::now();
                //     fk_solver.JntToCart(nominal, leg_contact_frame);
                //     for (size_t i = 0; i < number_of_joints; i++)
                //     {
                //         joint_state.name[i] = joint_name[i];
                //         joint_state.position[i] = nominal(i);
                //     }
                //     ROS_INFO("Leg is in default nominal state");
                //     print_frame_lambda(leg_contact_frame);
                // }
                // break;
        }

        joint_pub.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}