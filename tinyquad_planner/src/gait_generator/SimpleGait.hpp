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

            std::vector<geometry_msgs::Point> target_points_{};

            void constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, double scale, float colour);
            void generateCircle(geometry_msgs::Point origin, double radius, int plane);
            void generateRectangle(geometry_msgs::Point origin, double height, double width, int plane);
            void generateRectangle(std::vector<geometry_msgs::Point> points);
    };

} //simplegait_ns

#endif // header guard