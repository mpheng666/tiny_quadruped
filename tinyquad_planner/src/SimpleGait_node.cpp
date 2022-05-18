#include <ros/ros.h>
#include "gait_generator/SimpleGait.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_gait_node");
    ros::NodeHandle nh("~");

    // bezier_ns::Bezier bezier(&nh);
    // bezier.start();

    return 0;
}