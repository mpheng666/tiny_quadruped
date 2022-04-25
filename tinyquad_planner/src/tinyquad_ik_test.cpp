#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "sensor_msgs/Joy.h"
#include "urdf/model.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

const double PI = 3.14159;
KDL::JntArray joy_joints(4);
KDL::Frame leg_contact_frame;

double bias_x_nominal = -0.01;
double bias_y_nominal = 0.00;
double bias_z_nominal = 0.045;

double bias_x = 0.120024 + bias_x_nominal;
double bias_y = 0.050443 + bias_y_nominal;
double bias_z = -0.163388 + bias_z_nominal;

double scale_x = 0.1;
double scale_y = 0.1;
double scale_z = 0.05;

double x_end = 0.0f;
double y_end = 0.0f;
double z_end = 0.0f;

visualization_msgs::Marker marker;

enum class Mode
{
    cartesian_mode,
    joint_mode,
    bezier_mode,
    kinematics_mode
};

Mode leg_mode = Mode::cartesian_mode;

void joyCb(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons[0] == 1)
    {
        leg_mode = Mode::cartesian_mode;
    }
    else if (msg->buttons[1] == 1)
    {
        leg_mode = Mode::joint_mode;
    }
    else if (msg->buttons[2] == 1)
    {
        leg_mode = Mode::bezier_mode;
    }
    else if (msg->buttons[3] == 1)
    {
        leg_mode = Mode::kinematics_mode;
    }

    if (leg_mode == Mode::cartesian_mode)
    {
        marker.pose.position.x = msg->axes[0]*scale_x + bias_x;
        marker.pose.position.y = msg->axes[3]*scale_y + bias_y;
        marker.pose.position.z = msg->axes[1]*scale_z + bias_z;
        leg_contact_frame.p.x(marker.pose.position.x);
        leg_contact_frame.p.y(marker.pose.position.y);
        leg_contact_frame.p.z(marker.pose.position.z);
        marker.header.stamp = ros::Time::now();
    }
    else if (leg_mode == Mode::joint_mode)
    {
        joy_joints(0) = msg->axes[0];
        joy_joints(1) = msg->axes[1];
        joy_joints(2) = msg->axes[2];
        joy_joints(3) = msg->axes[3];
    }
    if (leg_mode == Mode::cartesian_mode)
    {
        marker.pose.position.x = msg->axes[0]*scale_x + bias_x;
        marker.pose.position.y = msg->axes[3]*scale_y + bias_y;
        marker.pose.position.z = msg->axes[1]*scale_z + bias_z;
        x_end = marker.pose.position.x;
        y_end = marker.pose.position.y;
        z_end = marker.pose.position.z;
        // leg_contact_frame.p.x(marker.pose.position.x);
        // leg_contact_frame.p.y(marker.pose.position.y);
        // leg_contact_frame.p.z(marker.pose.position.z);
        marker.header.stamp = ros::Time::now();
    }
}

void bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    leg_contact_frame.p.x(msg->point.x);
    leg_contact_frame.p.y(msg->point.y);
    leg_contact_frame.p.z(msg->point.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tq_ik_test_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_target_marker", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 100, &joyCb);
    // ros::Subscriber bezier_sub = n.subscribe<geometry_msgs::PointStamped>("bezier_points", 100, &bezierCb);
    ros::Rate loop_rate(20);

    static tf::TransformBroadcaster tf_marker;
    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    q.setRPY(0,0,0);
    tf.setRotation(q);
    tf_marker.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "leg_contact_frame"));

    // Marker for debugging
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "leg_contact_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
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
        tf_marker.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "leg_contact_frame"));

        switch (leg_mode)
        {
            // 1. Cartesian mode: listen joy::end_cartesian, calculate joints from inverse kinematics
            case Mode::cartesian_mode:
                {
                    joint_state.header.stamp = ros::Time::now();
                    int rc = tracik_solver.CartToJnt(curr_joint_array, leg_contact_frame, ik_result);
                    ROS_INFO("ik result status: %i", rc);
                    joint_state.name.resize(number_of_joints);
                    joint_state.position.resize(number_of_joints);

                    marker_pub.publish(marker);
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

                case Mode::kinematics_mode:
                {
                    joint_state.header.stamp = ros::Time::now();
                    double resultant_xyz    =  sqrt(    pow(x_end, 2.0f) + 
                                                        pow(y_end, 2.0f) + 
                                                        pow(z_end, 2.0f) );

                    double resultant_xz     =  sqrt(    pow(x_end, 2.0f) + 
                                                        pow(z_end, 2.0f) );

                    double resultant_yz     =  sqrt(    pow(y_end, 2.0f) + 
                                                        pow(z_end, 2.0f) );

                    joint_state.name.resize(number_of_joints);
                    joint_state.position.resize(number_of_joints);

                    marker_pub.publish(marker);
                    
                    for (size_t i = 0; i < number_of_joints; i++)
                    {
                        // ROS_INFO("ik_result: %f", double(ik_result(i)));
                        joint_state.name[i] = joint_name[i];
                        joint_state.position[i] = ik_result(i);
                        curr_joint_array.data[i] = ik_result(i);
                    }

                    print_frame_lambda(leg_contact_frame);
                    ROS_INFO("Leg is in kinematics mode");
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

            case Mode::bezier_mode:
                {
                    joint_state.header.stamp = ros::Time::now();
                    int rc = tracik_solver.CartToJnt(curr_joint_array, leg_contact_frame, ik_result);
                    ROS_INFO("ik result status: %i", rc);
                    joint_state.name.resize(number_of_joints);
                    joint_state.position.resize(number_of_joints);

                    marker_pub.publish(marker);
                    for (size_t i = 0; i < number_of_joints; i++)
                    {
                        ROS_INFO("ik_result: %f", double(ik_result(i)));
                        joint_state.name[i] = joint_name[i];
                        joint_state.position[i] = ik_result(i);
                        curr_joint_array.data[i] = ik_result(i);
                    }
                    print_frame_lambda(leg_contact_frame);
                    ROS_INFO("Leg is in bezier mode");
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