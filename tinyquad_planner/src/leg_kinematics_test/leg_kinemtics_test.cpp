// ros lib
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "kdl_parser/kdl_parser.hpp"

// standard lib
#include <iostream>
#include <string>
#include <math.h>
#include <vector>

// MACROS
#define SHOW(a) std::cout << #a << ": " << (a) << std::endl

inline double radtodeg(double rad) {return rad*180.0/M_PI;}
inline double degtorad(double deg) {return deg/180.0*M_PI;}

KDL::Tree robot_tree;
std::string robot_desc_string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_test_node");
    ros::NodeHandle nh;

    nh.param("robot_description", robot_desc_string, std::string());

    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    else
    {
        
    }

    // length of limbs
    double l_upper = 1.0;
    double l_lower = 1.0;

    // geometry angle
    double theta_1 = 0.0f;
    double theta_2 = 0.0f;
    double theta_3 = 0.0f;

    double x_end = 1.0f;
    double y_end = 1.0f;
    double z_end = 1.0f;

    std::vector<double> {x_end, y_end, z_end};

    // end effector pose
    // geometry_msgs::PoseStamped end_effector;
    // end_effector.header.frame_id = "end_effector_link";
    // end_effector.pose.position.x = 0.0;
    // end_effector.pose.position.y = 0.0;
    // end_effector.pose.position.z = 0.0;
    // end_effector.pose.orientation.x = 0.0;
    // end_effector.pose.orientation.y = 0.0;
    // end_effector.pose.orientation.z = 0.0;
    // end_effector.pose.orientation.w = 1.0;

    // Kinematics
    double resultant_xyz    =  sqrt(    pow(x_end, 2.0f) + 
                                        pow(y_end, 2.0f) + 
                                        pow(z_end, 2.0f) );

    double resultant_xz     =  sqrt(    pow(x_end, 2.0f) + 
                                        pow(z_end, 2.0f) );

    double resultant_yz     =  sqrt(    pow(y_end, 2.0f) + 
                                        pow(z_end, 2.0f) );

    double theta_xz = atan(x_end / z_end);                                        
    double theta_yz = atan(y_end / z_end);                                        
    double theta_yx = atan(y_end / x_end);

    SHOW(resultant_xyz);
    SHOW(resultant_xz);
    SHOW(resultant_yz);

    SHOW(radtodeg(theta_xz));
    SHOW(radtodeg(theta_yz));

    theta_3 = acos((resultant_xyz*resultant_xyz - l_upper*l_upper - l_lower*l_lower)
                            / -(2*l_upper*l_lower)
                            );

    theta_2 = (M_PI - theta_3) / 2.0f;

    SHOW(radtodeg(theta_2));
    SHOW(radtodeg(theta_3));

    return 0;
}