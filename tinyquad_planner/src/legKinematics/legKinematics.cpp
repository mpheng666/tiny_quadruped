#include "legKinematics.hpp"

namespace legKinematics_ns
{
    LegKinematics::LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror) :
    prefix(prefix),
    x_mirror(x_mirror),
    y_mirror(y_mirror),
    // spinner(4),
    joint_pub(p_nh_.advertise<sensor_msgs::JointState>("joint_states", 1)),
    marker_pub(p_nh_.advertise<visualization_msgs::Marker>("visualization_target_marker", 1)),
    bezier_sub(p_nh_.subscribe<geometry_msgs::PointStamped>("bezier_points", 100, &LegKinematics::bezierCb, this)),
    joy_sub(p_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &LegKinematics::joyCb, this)),
    tracik_solver(chain_start, chain_end, urdf_param, timeout, eps)
    {

    }
    LegKinematics::~LegKinematics()
    {
    }

    void LegKinematics::start()
    {
        ros::Rate r(LOOP_RATE);
        // spinner.start();

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
            createLeg();
        }
        // ros::waitForShutdown();
    }

    void LegKinematics::loadParam()
    {

    }

    void LegKinematics::initTf()
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "target_frame";
        tf.setOrigin( tf2::Vector3(0.0, 0.0, 0.0) );
        q.setRPY(0.0, 0.0, 0.0);
        tf.setRotation(q);

        tf_bc.sendTransform(transformStamped);
    }

    void LegKinematics::initMarker()
    {
        target_marker.header.frame_id = "foot_contact_frame";
        target_marker.header.stamp = ros::Time::now();

        target_marker.ns = "foot";
        target_marker.id = 0;
        target_marker.type = shape;

        target_marker.action = visualization_msgs::Marker::ADD;

        target_marker.pose.position.x = 0;
        target_marker.pose.position.y = 0;
        target_marker.pose.position.z = 0;
        target_marker.pose.orientation.x = 0.0;
        target_marker.pose.orientation.y = 0.0;
        target_marker.pose.orientation.z = 0.0;
        target_marker.pose.orientation.w = 1.0;

        target_marker.scale.x = 0.025;
        target_marker.scale.y = 0.025;
        target_marker.scale.z = 0.025;

        target_marker.color.r = 0.0f;
        target_marker.color.g = 1.0f;
        target_marker.color.b = 0.0f;
        target_marker.color.a = 1.0;

        target_marker.lifetime = ros::Duration();
    }

    void LegKinematics::createLeg()
    {
        for(std::string i :joint_name)
        {
            ROS_INFO_STREAM("Leg joint:" << i);
        }
    }

    void LegKinematics::inverseKinematics()
    {
        // given XYZ, calculate thetas
    }
    void LegKinematics::forwardKinematics()
    {
        // given thetas, calculate XYZ
    }

    void LegKinematics::joyCb(const sensor_msgs::Joy::ConstPtr& msg)
    {
        
    }

    void LegKinematics::bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {

    }

} // ns legKinematics_ns