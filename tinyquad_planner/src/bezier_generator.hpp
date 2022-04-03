#ifndef TINYQUAD_PLANNER_BEZIER_GENERATOR_
#define TINYQUAD_PLANNER_BEZIER_GENERATOR_

#include <math.h>

#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

#include <Eigen/Dense>

#include <vector>
#include <string>
namespace bezier_ns
{
    class Bezier
    {
        public:
            Bezier(ros::NodeHandle* nh);
            ~Bezier();
            void start();

        private:
            static constexpr double LOOP_RATE_ {20.0f};
            ros::NodeHandle private_nh_;
            ros::Publisher point_marker_pub_;
            ros::Publisher bezier_pub_;

            static tf::TransformBroadcaster points_tf;
            
            void loadParam();
            double factorial(int n);
            double comb(int n, int k);
            Eigen::MatrixXd getBezierMatrix(int n);
            std::vector<geometry_msgs::Point> evaluateBezier(std::vector<geometry_msgs::Point> points, double resolution);
            // void constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA colour);
            void constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, double scale, float colour);
    };

} // bezier_ns

#endif