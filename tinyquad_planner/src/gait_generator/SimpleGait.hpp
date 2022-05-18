#ifndef TINYQUAD_SIMPLEGAIT_HPP_
#define TINYQUAD_SIMPLEGAIT_HPP_

#include <ros/ros.h>
#include <iostream>

#include "geometry_msgs/Point.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

#include <Eigen/Dense>

#include <vector>

namespace simplegait_ns
{
    class SimpleGait
    {

        public:
            SimpleGait(ros::NodeHandle* nh);
            ~SimpleGait();
            void start();
        
        private:
            static constexpr double LOOP_RATE_ {20.0f};
            ros::NodeHandle private_nh_;

            ros::Publisher point_marker_pub_;
            static tf::TransformBroadcaster points_tf;

            geometry_msgs::Point p0;
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            geometry_msgs::Point p3;
            geometry_msgs::Point p4;
            geometry_msgs::Point p5;
            geometry_msgs::Point p6;

            std::vector<geometry_msgs::Point> target_points_{};

            void constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, double scale, float colour);

    };

} //simplegait_ns

#endif // header guard