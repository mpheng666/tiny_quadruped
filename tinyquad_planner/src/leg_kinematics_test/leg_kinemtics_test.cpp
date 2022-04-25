// ros lib
// #include "ros/ros.h"
// #include "geometry_msgs/PoseStamped.h"

// standard lib
#include <iostream>
#include <string>
#include <math.h>
#include <vector>

inline double radtodeg(double rad) {return rad*180.0/M_PI;}
inline double degtorad(double deg) {return deg/180.0*M_PI;}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "urdf_parser_node");

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


    printf("resultant_xyz: %f \n", resultant_xyz);    
    printf("resultant_xz: %f \n", resultant_xz);    
    printf("resultant_yz: %f \n", resultant_yz);


    theta_3 = acos((resultant_xyz*resultant_xyz - l_upper*l_upper - l_lower*l_lower)
                            / -(2*l_upper*l_lower)
                            );

    theta_1 = acos(z_end / resultant_yz);

    theta_2 = (M_PI/2.0f - theta_3) / 2.0f;

    printf("theta1_d: %f \n", radtodeg(theta_1));    
    printf("theta2_d: %f \n", radtodeg(theta_2));    
    printf("theta3_d: %f \n", radtodeg(theta_3));

    double theta_yx = tan(y_end / x_end);

    return 0;
}