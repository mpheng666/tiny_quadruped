#include "BodyKinematics.hpp"

using namespace bodykinematics_ns;

BodyKinematics::BodyKinematics(ros::NodeHandle& nh, std::string name):
rate_(LOOP_RATE_),
jointGroupCommandPub_(p_nh_.advertise<std_msgs::Float64MultiArray>("joint_group_command", 10)),
bodyCentroidPub_(p_nh_.advertise<geometry_msgs::PoseStamped>("body_pose", 10)),
markerPub_(p_nh_.advertise<visualization_msgs::Marker>("marker_pub", 10)),
joySub_(p_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &BodyKinematics::JoyCb, this))
{

}

BodyKinematics::~BodyKinematics()
{
    
}

void BodyKinematics::Start()
{
    auto target_marker = this->CreateMarker("COG_frame", "tinyquad", 1, 0, 0, 0, 0, 0, 0, 0, 1, 0.1);
    
    while(ros::ok())
    {
        // this->CalculateBodyInverseKinematics(target_marker.pose);
        this->ModifyMarker(target_marker, marker_pose_);
        markerPub_.publish(target_marker);
        ros::spinOnce();
        rate_.sleep();
    }
}

visualization_msgs::Marker BodyKinematics::CreateMarker(std::string frame_id, std::string ns, int id, int type, double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w, double scale, float r, float g, float b, float a)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = type;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = p_x;
    marker.pose.position.y = p_y;
    marker.pose.position.z = p_z;
    marker.pose.orientation.x = o_x;
    marker.pose.orientation.y = o_y;
    marker.pose.orientation.z = o_z;
    marker.pose.orientation.w = o_w;

    marker.scale.x = scale;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration();

    return marker;
}

void BodyKinematics::PublishMarker(const visualization_msgs::Marker& marker)
{

}

std::vector<geometry_msgs::Pose> BodyKinematics::CalculateBodyInverseKinematics(const geometry_msgs::Pose& bodyCentroid)
{
    std::vector<geometry_msgs::Pose> leg_poses;

    return leg_poses;
}

geometry_msgs::PoseStamped BodyKinematics::CalculateBodyForwardKinematics(const std::vector<geometry_msgs::Pose>& legs_pose)
{
    geometry_msgs::PoseStamped body_pose;

    return body_pose;
}

void BodyKinematics::JoyCb(const sensor_msgs::Joy::ConstPtr &msg)
{
    // hold button to move centroid
    if(msg->buttons[6] == 1)
    {
        double x = msg->axes[0] * scale_x_;
        double y = msg->axes[1] * scale_y_;
        double z = msg->axes[3] * scale_z_;
        marker_pose_.position.x = x;
        marker_pose_.position.y = y;
        marker_pose_.position.z = z;
    }
    // move RPY
    else
    {
        double roll = msg->axes[0] * scale_roll_;
        double pitch = msg->axes[1] * scale_pitch_;
        double yaw = msg->axes[2] * scale_yaw_;
        marker_pose_.orientation = (this->ConvertEulerToQuaternion(roll, pitch, yaw));
    }
}

void BodyKinematics::ModifyMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose pose)
{
    marker.pose = pose;
}

std::vector<double> BodyKinematics::ConvertQuaternionToEuler(const geometry_msgs::Quaternion& quat_msg)
{
    tf2::Quaternion quat_tf(quat_msg.x,
                            quat_msg.y,
                            quat_msg.z,
                            quat_msg.w);
    tf2::Matrix3x3 m(quat_tf);

    std::vector<double> rpy;
    m.getRPY(rpy[0], rpy[1], rpy[2]);

    return rpy;
}

geometry_msgs::Quaternion BodyKinematics::ConvertEulerToQuaternion(const double& roll, const double& pitch, const double& yaw)
{
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    quat_tf.normalize();

    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    tf2::convert(quat_msg , quat_tf);

    return quat_msg;
}