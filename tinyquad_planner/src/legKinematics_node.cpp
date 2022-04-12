#include "legKinematics/legKinematics.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_kinematics_node");
    ros::NodeHandle nh("~");
    std::string prefix_lf {"LF"};
    std::string prefix_rf {"RF"};
    std::string prefix_lb {"LB"};
    std::string prefix_rb {"RB"};

    legKinematics_ns::LegKinematics leg_lf(nh, prefix_lf, 1, 1);
    legKinematics_ns::LegKinematics leg_rf(nh, prefix_rf, -1, 1);
    legKinematics_ns::LegKinematics leg_lb(nh, prefix_lb, 1, -1);
    legKinematics_ns::LegKinematics leg_rb(nh, prefix_rb, -1, -1);

    leg_lf.start();
    leg_rf.start();
    leg_lb.start();
    leg_rb.start();

    return 0;
}