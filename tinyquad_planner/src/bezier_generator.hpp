#ifndef TINYQUAD_PLANNER_BEZIER_GENERATOR_
#define TINYQUAD_PLANNER_BEZIER_GENERATOR_

#include <math.h>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

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
            ros::NodeHandle private_nh_;
            static constexpr double LOOP_RATE_ {20.0f};
            ros::Publisher point_marker_pub_;

            static tf::TransformBroadcaster points_tf;
            
            std::vector<double> Points;
            double f {0.0f};

            void loadParam();
            // visualization_msgs::Marker constructMarker(uint32_t shape, std::string frame_id, ros::Time stamp, );
    };

} // bezier_ns

#endif