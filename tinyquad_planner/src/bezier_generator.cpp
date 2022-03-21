#include "bezier_generator.hpp"

namespace bezier_ns
{

    Bezier::Bezier(ros::NodeHandle* nh) :
    private_nh_(*nh),
    point_marker_pub_(nh->advertise<visualization_msgs::Marker>("visualization_marker", 1))
    {
        ROS_INFO("Bezier node constructor initiated!");
    }

    Bezier::~Bezier()
    {

    }

    void Bezier::start()
    {
        ros::Rate r(LOOP_RATE_);

        while(private_nh_.ok())
        {

            visualization_msgs::Marker point_1, point_2, point_3, curve_point;
            point_1.header.frame_id = point_2.header.frame_id = point_3.header.frame_id = curve_point.header.frame_id = "my_frame";
            point_1.header.stamp = point_2.header.stamp = point_3.header.stamp = curve_point.header.stamp = ros::Time::now();
            point_1.ns = point_2.ns = point_3.ns = curve_point.ns = "bezier";
            point_1.action = point_2.action = point_3.action = curve_point.action = visualization_msgs::Marker::ADD;
            point_1.pose.orientation.w = point_2.pose.orientation.w = point_3.pose.orientation.w = curve_point.pose.orientation.w = 1.0;

            point_1.id = 0;
            point_2.id = 1;
            point_3.id = 2;
            curve_point.id = 3;

            point_1.type = visualization_msgs::Marker::SPHERE;
            point_2.type = visualization_msgs::Marker::SPHERE;
            point_3.type = visualization_msgs::Marker::SPHERE;
            curve_point.type = visualization_msgs::Marker::POINTS;

            // point_1 markers use x and y scale for width/height respectively
            point_1.scale.x = 0.2;
            point_1.scale.y = 0.2;
            point_1.scale.z = 0.2;

            point_2.scale.x = 0.2;
            point_2.scale.y = 0.2;
            point_2.scale.z = 0.2;

            point_3.scale.x = 0.2;
            point_3.scale.y = 0.2;
            point_3.scale.z = 0.2;

            curve_point.scale.x = 0.01;
            curve_point.scale.y = 0.01;
            curve_point.scale.z = 0.01;

            point_1.color.g = 1.0;
            point_1.color.a = 1.0;

            point_2.color.g = 1.0;
            point_2.color.a = 1.0;

            point_3.color.g = 1.0;
            point_3.color.a = 1.0;

            curve_point.color.r = 1.0;
            curve_point.color.a = 1.0;

            point_1.pose.position.x = 0.0;
            point_1.pose.position.y = 0.0;
            point_1.pose.position.z = 0.0;

            point_2.pose.position.x = 1.0;
            point_2.pose.position.y = -0.5;
            point_2.pose.position.z = 0.0;

            point_3.pose.position.x = 2.0;
            point_3.pose.position.y = 1.0;
            point_3.pose.position.z = 0.0;

            point_marker_pub_.publish(point_1);
            point_marker_pub_.publish(point_2);
            point_marker_pub_.publish(point_3);

            for (double u = 0; u < 1 ; u+= 0.01)
            {
                // curve_point.pose.position.x = pow(1-u, 2)*point_1.pose.position.x + 2*(1-u)*u*point_2.pose.position.x + pow(u, 2)*point_3.pose.position.x;
                // curve_point.pose.position.y = pow(1-u, 2)*point_1.pose.position.y + 2*(1-u)*u*point_2.pose.position.y + pow(u, 2)*point_3.pose.position.y;
                point_marker_pub_.publish(point_1);
                point_marker_pub_.publish(point_2);
                point_marker_pub_.publish(point_3);

                geometry_msgs::Point p;
                p.x = pow(1-u, 2)*point_1.pose.position.x + 2*(1-u)*u*point_2.pose.position.x + pow(u, 2)*point_3.pose.position.x;
                p.y = pow(1-u, 2)*point_1.pose.position.y + 2*(1-u)*u*point_2.pose.position.y + pow(u, 2)*point_3.pose.position.y;
                curve_point.points.push_back(p);
                curve_point.pose.position.z = 0.0;
                point_marker_pub_.publish(curve_point);
                r.sleep();
            }

            r.sleep();
            ros::spinOnce();
        }

    }

    void Bezier::loadParam()
    {

    }

    // visualization_msgs::Marker Bezier::constructMarker(uint32_t shape, )
    // {

    // }
            

} // bezier_ns