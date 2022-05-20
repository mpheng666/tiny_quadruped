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

void BodyKinematics::DefineDefaultVariables()
{
    defaultPose_.orientation.w = 1.0;
    defaultColour_.g = 1.0;
    defaultColour_.a = 1.0;
}

void BodyKinematics::LoadParam()
{
    bodyCorners_.resize(4);
    for(int i=0; i<bodyCorners_.size(); ++i)
    {
        std::string param_name = "body_corners/point_" + std::to_string(i) + "_";
        p_nh_.param<double>(param_name + "x", bodyCorners_[i].x, 0.0); 
        p_nh_.param<double>(param_name + "y", bodyCorners_[i].y, 0.0); 
        p_nh_.param<double>(param_name + "z", bodyCorners_[i].z, 0.0); 
    }
}

void BodyKinematics::Start()
{
    this->DefineDefaultVariables();
    this->LoadParam();
    auto markers_rectangle = this->DrawRecangle(bodyCorners_);
    auto target_marker = this->CreateMarker("test_frame", "tinyquad", 2, visualization_msgs::Marker::ARROW, defaultPose_, 1, defaultColour_);
    
    while(ros::ok())
    {
        // this->CalculateBodyInverseKinematics(target_marker.pose);
        this->ModifyMarker(target_marker, marker_pose_);
        markerPub_.publish(target_marker);
        this->PublishMarker(markers_rectangle);
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

    if(marker.type == 0)
    {
        marker.scale.x = scale;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
    }
    else
    {
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
    }

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration();

    return marker;
}

visualization_msgs::Marker BodyKinematics::CreateMarker(std::string frame_id, std::string ns, int id, int type, geometry_msgs::Pose& pose, double scale, std_msgs::ColorRGBA& colour)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = type;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;
    if(marker.type == 0)
    {
        marker.scale.x = scale;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
    }
    else
    {
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
    }

    marker.color = colour;

    marker.lifetime = ros::Duration();

    return marker;
}

void BodyKinematics::PublishMarker(const visualization_msgs::Marker& marker)
{
    ros::Publisher marker_pub = p_nh_.advertise<visualization_msgs::Marker>("marker_pub", 10);
    marker_pub.publish(marker);
}

void BodyKinematics::PublishMarker(const std::vector<visualization_msgs::Marker>& markers)
{

    for(auto i=0; i<markers.size(); ++i)
    {
        markerPub_.publish(markers.at(i));
    }
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
        double x = msg->axes[0] * SCALE_X_;
        double y = msg->axes[1] * SCALE_Y_;
        double z = msg->axes[3] * SCALE_Z_;
        marker_pose_.position.x = x;
        marker_pose_.position.y = y;
        marker_pose_.position.z = z;
    }
    // move RPY
    else
    {
        double roll = msg->axes[0] *    SCALE_ROLL_;
        double pitch = msg->axes[1] *   SCALE_PITCH_;
        double yaw = msg->axes[2] *     SCALE_YAW_;
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

geometry_msgs::Quaternion BodyKinematics::ConvertEulerToQuaternion(const double roll, const double pitch, const double yaw)
{
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    quat_tf.normalize();

    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    tf2::convert(quat_msg , quat_tf);

    return quat_msg;
}

std::vector<visualization_msgs::Marker> BodyKinematics::DrawRecangle(std::vector<geometry_msgs::Point>& points)
{
    std::vector<visualization_msgs::Marker> markers;
    auto lines_marker = this->CreateMarker("rec", "tinyquad", 4, visualization_msgs::Marker::LINE_STRIP, defaultPose_, 0.01, defaultColour_);

    for(auto i=0; i<points.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;

        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.a = 1.0;

        auto points_marker = this->CreateMarker("rec", "tinyquad", i, visualization_msgs::Marker::POINTS, defaultPose_, 0.1, red);
        lines_marker.points.push_back(p);
        points_marker.points.emplace_back(p);
        markers.emplace_back(points_marker);
    }

    lines_marker.points.emplace_back(points.at(0));
    markers.emplace_back(lines_marker);

    return markers;
}
