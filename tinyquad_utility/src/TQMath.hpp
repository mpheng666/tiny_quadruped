#ifndef _TINYQUAD_TQVISUALIZE_HPP_
#define _TINYQUAD_TQVISUALIZE_HPP_

#include <vector>
#include <geometry_msgs/Quaternion.h>

namespace TQmath_ns
{
    class TQMath
    {
        public:
            std::vector<double> ConvertQuaternionToEuler(const geometry_msgs::Quaternion& q);
            geometry_msgs::Quaternion ConvertEulerToQuaternion(const double roll, const double pitch, const double yaw);
    };

}

#endif

