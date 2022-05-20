#include "body_kinematics/BodyKinematics.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_kinematics_node");
    ros::NodeHandle nh("~");
    bodykinematics_ns::BodyKinematics tinyquad_1(nh, "tinyquad_body");
    tinyquad_1.Start();

    return 0;
}