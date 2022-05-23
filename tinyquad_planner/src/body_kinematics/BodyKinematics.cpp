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
    bodyCentroid_.pose.orientation = defaultPose_.orientation;
    bodyCornersInitial_ = bodyCorners_;
}

void BodyKinematics::LoadParam()
{
    p_nh_.param<bool>("use_points", usePoints_, false);
    bodyCorners_.resize(4);
    for(int i=0; i<bodyCorners_.size(); ++i)
    {
        std::string param_name = "body_corners/point_" + std::to_string(i) + "_";
        p_nh_.param<double>(param_name + "x", bodyCorners_[i].x, 0.0); 
        p_nh_.param<double>(param_name + "y", bodyCorners_[i].y, 0.0); 
        p_nh_.param<double>(param_name + "z", bodyCorners_[i].z, 0.0); 
    }
    p_nh_.param<double>("body_size/length", bodyLength_, 0.5);
    p_nh_.param<double>("body_size/width", bodyWidth_, 0.25);

    if(usePoints_)
    {
        auto bodysize = this->FindSizeFromPoints(bodyCorners_);
        bodyLength_ = bodysize.at(0);
        bodyWidth_ = bodysize.at(1);
    }
    else
    {
        std::vector<double> body;
        body.emplace_back(bodyLength_);
        body.emplace_back(bodyWidth_);
        bodyCorners_ = this->FindPointsFromSize(body);
    }
}

void BodyKinematics::Start()
{
    this->LoadParam();
    this->DefineDefaultVariables();
    auto markers_rectangle = this->DrawRecangle(bodyCorners_);
    auto target_marker = this->CreateMarker("TQ_frame", "tinyquad", 10, visualization_msgs::Marker::ARROW, defaultPose_, 1, defaultColour_);
    
    while(ros::ok())
    {
        this->UpdateMarker(target_marker, markerPose_);

        this->CalculateBodyInverseKinematics(markerPose_, bodyCorners_);
        this->DrawRecangle(bodyCorners_);
        ros::spinOnce();
        rate_.sleep();
    }
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
    // ros::Publisher marker_pub = p_nh_.advertise<visualization_msgs::Marker>("marker_pub", 10);
    markerPub_.publish(marker);
}

void BodyKinematics::PublishMarker(const std::vector<visualization_msgs::Marker>& markers)
{

    for(auto i=0; i<markers.size(); ++i)
    {
        markerPub_.publish(markers.at(i));
    }
}

// only calcalate when there is changes to improve efficiency
// Doing what tf is supposed to do if frames are defined properly
std::vector<geometry_msgs::Point> BodyKinematics::CalculateBodyInverseKinematics(const geometry_msgs::Pose& center_target, std::vector<geometry_msgs::Point>& points)
{
    std::vector<double> rpy = this->ConvertQuaternionToEuler(markerPose_.orientation);
    std::vector<double> rpyPrev = this->ConvertQuaternionToEuler(markerPosePrevious_.orientation);

    // for(const auto i : rpy)
    // {
    //     ROS_INFO_STREAM("angle: " << i);        
    // }

    double delta_x = markerPose_.position.x - markerPosePrevious_.position.x;
    double delta_y = markerPose_.position.y - markerPosePrevious_.position.y;
    double delta_z = markerPose_.position.z - markerPosePrevious_.position.z;

    for(int i=0; i<points.size(); ++i)
    {
        points.at(i).x += delta_x;
        points.at(i).y += delta_y;
        points.at(i).z += delta_z;
    }

    // All deltas are respected to point 0
    // ROLL YZ
    double current_y_roll = -(bodyWidth_/2.0 - bodyWidth_/2.0 * cos(rpy[0])); 
    double previous_y_roll = -(bodyWidth_/2.0 - bodyWidth_/2.0 * cos(rpyPrev[0])); 
    double delta_y_roll = current_y_roll - previous_y_roll; 
    double current_z_roll = -(bodyWidth_/2.0 * sin(rpy[0]));
    double previous_z_roll = -(bodyWidth_/2.0 * sin(rpyPrev[0]));
    double delta_z_roll = current_z_roll - previous_z_roll;
    points.at(0).y += delta_y_roll;
    points.at(0).z += delta_z_roll;
    points.at(1).y += delta_y_roll;
    points.at(1).z += delta_z_roll;
    points.at(2).y -= delta_y_roll;
    points.at(2).z -= delta_z_roll;    
    points.at(3).y -= delta_y_roll;
    points.at(3).z -= delta_z_roll;

    // PITCH XZ
    double current_x_pitch = -(bodyLength_/2.0 - bodyLength_/2.0 * cos(rpy[1])); 
    double previous_x_pitch = -(bodyLength_/2.0 - bodyLength_/2.0 * cos(rpyPrev[1])); 
    double delta_x_pitch = current_x_pitch - previous_x_pitch; 
    double current_z_pitch = -(bodyLength_/2.0 * sin(rpy[1]));
    double previous_z_pitch = -(bodyLength_/2.0 * sin(rpyPrev[1]));
    double delta_z_pitch = current_z_pitch - previous_z_pitch;
    points.at(0).x += delta_x_pitch;
    points.at(0).z += delta_z_pitch;
    points.at(1).x -= delta_x_pitch;
    points.at(1).z -= delta_z_pitch;
    points.at(2).x -= delta_x_pitch;
    points.at(2).z -= delta_z_pitch;    
    points.at(3).x += delta_x_pitch;
    points.at(3).z += delta_z_pitch;
    
    // double angle_debug = atan(points.at(0).z / points.at(0).x);
    // ROS_INFO_STREAM("debug angle: " << angle_debug);

    // YAW XY
    // ROS_INFO("YAW: %f \n", rpy[2]);
    points.at(0).x = bodyCornersInitial_.at(0).x * cos(-rpy[2]) + bodyCornersInitial_.at(0).y * sin(-rpy[2]);
    points.at(0).y = bodyCornersInitial_.at(0).y * cos(-rpy[2]) - bodyCornersInitial_.at(0).x * sin(-rpy[2]);
    points.at(1).x = bodyCornersInitial_.at(1).x * cos(-rpy[2]) + bodyCornersInitial_.at(1).y * sin(-rpy[2]);
    points.at(1).y = bodyCornersInitial_.at(1).y * cos(-rpy[2]) - bodyCornersInitial_.at(1).x * sin(-rpy[2]);
    points.at(2).x = bodyCornersInitial_.at(2).x * cos(-rpy[2]) + bodyCornersInitial_.at(2).y * sin(-rpy[2]);
    points.at(2).y = bodyCornersInitial_.at(2).y * cos(-rpy[2]) - bodyCornersInitial_.at(2).x * sin(-rpy[2]);
    points.at(3).x = bodyCornersInitial_.at(3).x * cos(-rpy[2]) + bodyCornersInitial_.at(3).y * sin(-rpy[2]);
    points.at(3).y = bodyCornersInitial_.at(3).y * cos(-rpy[2]) - bodyCornersInitial_.at(3).x * sin(-rpy[2]);
    // ROS_INFO("x[0]: %f \n", points.at(0).x);
    // ROS_INFO("y[0]: %f \n", points.at(0).y);

    markerPosePrevious_ = markerPose_;

    return points;
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
        markerPose_.position.x = x;
        markerPose_.position.y = y;
        markerPose_.position.z = z;
    }
    // move RPY
    else
    {
        double roll = msg->axes[0] *    SCALE_ROLL_;
        double pitch = msg->axes[1] *   -SCALE_PITCH_;
        double yaw = msg->axes[2] *     SCALE_YAW_;
        markerPose_.orientation = (this->ConvertEulerToQuaternion(roll, pitch, yaw));
    }
}

void BodyKinematics::UpdateMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose pose)
{
    marker.pose = pose;
    this->PublishMarker(marker);
}


std::vector<double> BodyKinematics::ConvertQuaternionToEuler(const geometry_msgs::Quaternion& quat_msg)
{
    
    tf2::Quaternion quat_tf(quat_msg.x,
                            quat_msg.y,
                            quat_msg.z,
                            quat_msg.w);

    // ROS_INFO_STREAM("quat: " << quat_msg.x);
    // ROS_INFO_STREAM("quat: " << quat_msg.y);
    // ROS_INFO_STREAM("quat: " << quat_msg.z);
    // ROS_INFO_STREAM("quat: " << quat_msg.w);

    tf2::Matrix3x3 m(quat_tf);

    std::vector<double> rpy;
    rpy.resize(3);
    m.getRPY(rpy[0], rpy[1], rpy[2]);

    for(int i=0; i<rpy.size(); ++i)
    {   
        if(isnan(rpy[i]))
        {
            ROS_WARN("NO EULER SOLUTION FOUND, return ZEROs");
            std::fill(rpy.begin(), rpy.end(), 0);
            return rpy;
        }
        else
        {
            // ROS_INFO_STREAM("q2e [" << i << "] :" << rpy[i]);
        }
    }

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
    auto lines_marker = this->CreateMarker("TQ_frame", "tinyquad", 4, visualization_msgs::Marker::LINE_STRIP, defaultPose_, 0.01, defaultColour_);

    for(auto i=0; i<points.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;

        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.a = 1.0;

        auto points_marker = this->CreateMarker("TQ_frame", "tinyquad", i, visualization_msgs::Marker::POINTS, defaultPose_, 0.1, red);
        lines_marker.points.push_back(p);
        points_marker.points.emplace_back(p);
        markers.emplace_back(points_marker);
    }

    lines_marker.points.emplace_back(points.at(0));
    markers.emplace_back(lines_marker);

    this->PublishMarker(markers);

    return markers;
}

std::vector<geometry_msgs::Point> BodyKinematics::FindPointsFromSize(std::vector<double>& size)
{
    double l = size.at(0);
    double w = size.at(1);
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point point;
    point.x = l/2.0;
    point.y = w/2.0;
    points.emplace_back(point);
    point.x = -point.x;
    points.emplace_back(point);
    point.y = -point.y;
    points.emplace_back(point);
    point.x = -point.x;
    points.emplace_back(point);

    return points;
}

std::vector<double> BodyKinematics::FindSizeFromPoints(std::vector<geometry_msgs::Point>& points)
{
    std::vector<double> size;
    double l = abs(points.at(0).x - points.at(1).x);
    double w = abs(points.at(1).y - points.at(2).y);
    size.emplace_back(l);
    size.emplace_back(w);

    return size;
}
