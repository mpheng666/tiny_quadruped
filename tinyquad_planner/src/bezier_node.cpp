#include <ros/ros.h>
#include "bezier_generator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bezier_node");
    ros::NodeHandle nh("~");

    bezier_ns::Bezier bezier(&nh);
    bezier.start();

    return 0;
}