#include "BodyKinematics.hpp"

namespace bodykinematics_ns
{

    BodyKinematics::BodyKinematics(ros::NodeHandle& nh, std::string name):
    rate(LOOP_RATE),
    jointGroupCommandPub(p_nh_.advertise<std_msgs::Float64MultiArray>("joint_group_command", 10)),
    bodyCentroidPub(p_nh_.advertise<geometry_msgs::PoseStamped>("body_pose", 10)),
    joySub(p_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &BodyKinematics::JoyCb, this))
    {

    }

    BodyKinematics::~BodyKinematics()
    {
        
    }

    void BodyKinematics::Start()
    {
        
    }

    void BodyKinematics::JoyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {

    }
    
}