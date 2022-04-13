#include "legKinematics/legKinematics.hpp"
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leg_kinematics_node");
    ros::NodeHandle nh("~");
    std::string prefix_lf {"LF"};
    std::string prefix_rf {"RF"};
    std::string prefix_lb {"LB"};
    std::string prefix_rb {"RB"};

    legKinematics_ns::LegKinematics* legPtr_lf = new legKinematics_ns::LegKinematics(nh, prefix_lf, 1, 1);
    // legKinematics_ns::LegKinematics* legPtr_rf = new legKinematics_ns::LegKinematics(nh, prefix_rf, -1, 1);
    // legKinematics_ns::LegKinematics* legPtr_lb = new legKinematics_ns::LegKinematics(nh, prefix_lb, 1, -1);
    // legKinematics_ns::LegKinematics* legPtr_rb = new legKinematics_ns::LegKinematics(nh, prefix_rb, -1, -1);
    
    // legKinematics_ns::LegKinematics leg_lf(nh, prefix_lf, 1, 1);
    // legKinematics_ns::LegKinematics leg_rf(nh, prefix_rf, -1, 1);
    // legKinematics_ns::LegKinematics leg_lb(nh, prefix_lb, 1, -1);
    // legKinematics_ns::LegKinematics leg_rb(nh, prefix_rb, -1, -1);

    std::thread t1(&legKinematics_ns::LegKinematics::start, legPtr_lf);
    // std::thread t2(&legKinematics_ns::LegKinematics::start, legPtr_rf);
    // std::thread t3(&legKinematics_ns::LegKinematics::start, legPtr_lb);
    // std::thread t4(&legKinematics_ns::LegKinematics::start, legPtr_rb);

    t1.join();
    // t2.join();
    // t3.join();
    // t4.join();


    // std::thread t1(leg_lf.start());
    // leg_lf.start();
    // leg_rf.start();
    // leg_lb.start();
    // leg_rb.start();

    return 0;
}