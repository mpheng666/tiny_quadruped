#include "bezier_generator.hpp"

namespace bezier_ns
{

    Bezier::Bezier(ros::NodeHandle* nh) :
    private_nh_(*nh),
    point_marker_pub_(nh->advertise<visualization_msgs::Marker>("visualization_marker", 1)),
    bezier_pub_(nh->advertise<geometry_msgs::PointStamped>("beizer_points", 1))
    {
        // ROS_INFO("Bezier node constructor initiated!");
    }

    Bezier::~Bezier()
    {

    }

    void Bezier::start()
    {
        ros::Rate r(LOOP_RATE_);

        std::vector<geometry_msgs::Point> ac_points;
        std::vector<geometry_msgs::Point> bc_points;

        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        geometry_msgs::Point p3;
        geometry_msgs::Point p4;
        geometry_msgs::Point p5;
        geometry_msgs::Point p6;

        p1.x = -0.6;
        p1.y = 0.0;
        p1.z = 1.0;
        
        p2.x = 1.0;
        p2.y = 0.6;
        p2.z = -0.3;

        p3.x = 1.0;
        p3.y = -3.0;
        p3.z = 0.0;
        
        p4.x = -4.0;
        p4.y = 2.0;
        p4.z = 3.0;

        p5.x = -1.0;
        p5.y = 2.2;
        p5.z = 5.0;

        p6.x = 4.54;
        p6.y = 2.0;
        p6.z = 2.0;
        
        ac_points.push_back(p1);
        ac_points.push_back(p2);
        ac_points.push_back(p3);
        ac_points.push_back(p4);
        ac_points.push_back(p5);
        ac_points.push_back(p6);

        visualization_msgs::Marker marker1;
        
        marker1.type = visualization_msgs::Marker::POINTS;
        marker1.header.frame_id = "bz_frame";
        marker1.header.stamp = ros::Time::now();
        marker1.ns = "bz_ns";
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.orientation.w = 1.0;
        marker1.id = 1;
        marker1.scale.x = 0.2;
        marker1.scale.y = 0.2;
        marker1.scale.z = 0.2;
        marker1.color.a = 1.0;
        marker1.color.g = 1.0;
        marker1.lifetime = ros::Duration(0);
        marker1.points.push_back(p1);
        marker1.points.push_back(p2);
        marker1.points.push_back(p3);
        marker1.points.push_back(p4);
        marker1.points.push_back(p5);
        marker1.points.push_back(p6);

        point_marker_pub_.publish(marker1);

        visualization_msgs::Marker bezier_marker;

        bool matrix_flag = true;

        while(private_nh_.ok())
        {
            point_marker_pub_.publish(marker1);

            if(matrix_flag)
            {
                bc_points = evaluateBezier(ac_points, 0.01f);
                matrix_flag = false;
            }

            for(int i=0; i<bc_points.size(); ++i)
            {
                bezier_marker.type = visualization_msgs::Marker::POINTS;
                bezier_marker.header.frame_id = "bz_frame";
                bezier_marker.header.stamp = ros::Time::now();
                bezier_marker.ns = "bz_ns";
                bezier_marker.action = visualization_msgs::Marker::ADD;
                bezier_marker.id = 88;
                bezier_marker.scale.x = 0.1;
                bezier_marker.scale.y = 0.1;
                bezier_marker.scale.z = 0.1;
                bezier_marker.color.a = 1.0;
                bezier_marker.color.r = 1.0;
                bezier_marker.points.push_back(bc_points.at(i));
                point_marker_pub_.publish(bezier_marker);
                r.sleep();
            }

            bezier_marker.points.clear();
            // matrix_flag = true;

            r.sleep();
            ros::spinOnce();
        }

    }

    void Bezier::loadParam()
    {

    }

    /**
     * @brief return factorial of n
     * 
     * @param n 
     * @return double 
     */
    double Bezier::factorial(int n)
    {
        double fact_result = 1.0;
        // printf("fact n: %i \n", n);
        if(n < 0)
        {
            ROS_INFO("Cannot factorial negative numbers");
        } else
        {
            for(int i=1; i<=n; ++i)
            {
                fact_result *= i;
                // printf("fact result: %f", fact_result);
            }
            // ROS_INFO("Factorial of n is: %f", fact_result);
        }
        return fact_result;
    }

    /**
     * @brief get coefficient of matrix of bezier based on bernstein polynomial
     * 
     * @param n 
     * @param k 
     * @return double 
     */
    double Bezier::comb(int n, int k)
    {
        return (this->factorial(n) / ((this->factorial(k)) * this->factorial(n-k)));
    }

    /**
     * @brief dynamically obtain bezier matrix based on the order of polynomial
     * 
     * @param n 
     * @return Eigen::MatrixXf 
     */
    Eigen::MatrixXd Bezier::getBezierMatrix(int n)
    {
        int size_of_mat = n+1;
        Eigen::MatrixXd coef_mat= Eigen::MatrixXd::Zero(size_of_mat, size_of_mat);

        for(int i=0; i<(size_of_mat); ++i)
        {
            for(int k=0 ; k<(i+1); ++k)
            {
                // printf("i: %i \t", i);
                // printf("k: %i \n", k);
                coef_mat(i,k) = comb(n, i) * comb(i, k) * pow(-1, (i-k));
            }
        }
        // std::cout << "Coef M: \n" << coef_mat << std::endl;
        return coef_mat;
    }

    // return T*M*P
    std::vector<geometry_msgs::Point> Bezier::evaluateBezier(std::vector<geometry_msgs::Point> &anchor_points, double resolution)
    {
        std::vector<geometry_msgs::Point> bcurve_points;
        int n = anchor_points.size();

        // vector<points> to matrixx3d
        Eigen::MatrixXd P(n,3);
        for (int i=0; i<n; ++i)
        {
            // ROS_INFO("vector to mat: %i", i);
            P(i,0) = anchor_points.at(i).x;
            P(i,1) = anchor_points.at(i).y;
            P(i,2) = anchor_points.at(i).z;
        }

        // std::cout << "P: \n" << P << std::endl;

        Eigen::MatrixXd M = getBezierMatrix(n-1);

        for (double t=0; t<1; t+=resolution)
        {
            // derive T based on t
            Eigen::RowVectorXd T(n);
            for (int i=0; i<n; ++i)
            {
                // ROS_INFO("T i: %i", i);
                // ROS_INFO("T t: %f", t);
                T(i) = pow(t, i);
                // std::cout << "T(i):" << T(i) << std::endl;
            }
            // std::cout << "T:" << T << std::endl;

            auto tmp = T * M * P;
            // std::cout << "bcurve_point: \n" << tmp << std::endl;
            geometry_msgs::Point bcurve_point;
            bcurve_point.x = tmp(0);
            bcurve_point.y = tmp(1);
            bcurve_point.z = tmp(2);
            bcurve_points.push_back(bcurve_point);
        }
        return bcurve_points;
    }

    // void Bezier::constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA colour)
    void Bezier::constructMarker(visualization_msgs::Marker marker, int32_t shape, std::string frame_id, std::string ns, geometry_msgs::Point point, int id, double scale, float colour)
    {
        std::cout << "frame id:" << frame_id << std::endl;
        marker.type = shape;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point;
        marker.id = id;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.a = colour;
        marker.color.g = colour;
        marker.lifetime = ros::Duration(0);
    }
            

} // bezier_ns