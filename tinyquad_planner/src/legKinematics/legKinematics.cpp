#include "legKinematics.hpp"

namespace legKinematics_ns
{
    LegKinematics::LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror) :
    prefix_(prefix),
    x_mirror_(x_mirror),
    y_mirror_(y_mirror),
    joint_pub_(p_nh_.advertise<sensor_msgs::JointState>("joint_state_s", 1)),
    marker_pub_(p_nh_.advertise<visualization_msgs::Marker>("visualization_target_marker_", 1)),
    cmd_vel_pub_(p_nh_.advertise<geometry_msgs::Twist>("/tinyquad/wheels_controller/cmd_vel", 1)),
    bezier_sub_(p_nh_.subscribe<geometry_msgs::PointStamped>("bezier_points", 100, &LegKinematics::bezierCb, this)),
    joy_sub_(p_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &LegKinematics::joyCb, this)),
    imu_sub_(p_nh_.subscribe<sensor_msgs::Imu>("/tinyquad/imu", 100, &LegKinematics::imuCb, this)),
    joy_joints_(5),
    tracik_solver_(chain_start_, chain_end_, urdf_param_, timeout_, eps_),
    fk_solver_(chain_)
    {

    }
    LegKinematics::~LegKinematics()
    {
    }

    void LegKinematics::start()
    {
        ros::Rate r(LOOP_RATE_);
        ros::Rate init_r(0.5);

        // init_r.sleep();
        this->loadParam();
        this->initLegPubs();
        this->createLeg();
        this->initTf("base_link", "foot_contact_frame");
        this->initMarker("foot_contact_marker", "foot", 0, visualization_msgs::Marker::CUBE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.025, marker_colour_.RED);

        if(this->checkKDLChain()) 
        {
            lower_limit_ = this->getKDLLimits().at(0);
            upper_limit_ = this->getKDLLimits().at(1);
            number_of_joints_ = this->getNumberOfJoints(chain_);
            // this->initForwardKinematics();
            curr_joint_array_ = this->getJointsNominal(lower_limit_, upper_limit_);
        }

        while(ros::ok())
        {
            this->updateFSM();
            joint_pub_.publish(joint_state_);
            ros::spinOnce();
            r.sleep();
        }
    }

    void LegKinematics::loadParam()
    {

    }

    void LegKinematics::initLegPubs()
    {
        legPubs_ = std::vector<ros::Publisher>(5);

        for(int i=0; i < legPubs_.size(); ++i)
        {
            legPubs_.at(i) = p_nh_.advertise<std_msgs::Float64>
            (robot_namespace_ + joint_name_.at(i) + "_position_controller/command", 10);
        }
    }

    void LegKinematics::initTf(std::string frame_id, std::string child_frame_id)
    {
        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = frame_id;
        transformStamped_.child_frame_id = child_frame_id;
        tf_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        q_.setRPY(0.0, 0.0, 0.0);
        tf_.setRotation(q_);
        transformStamped_.transform.translation.x = 0.0f;
        transformStamped_.transform.translation.y = 0.0f;
        transformStamped_.transform.translation.z = 0.0f;
        transformStamped_.transform.rotation.x = q_.x();
        transformStamped_.transform.rotation.y = q_.y();
        transformStamped_.transform.rotation.z = q_.z();
        transformStamped_.transform.rotation.w = q_.w();
        tf_bc_.sendTransform(transformStamped_);
    }

    void LegKinematics::initMarker(std::string frame_id, std::string ns, int id, int type, double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w, double scale, float colour[4])
    {
        target_marker_.header.frame_id = frame_id;
        target_marker_.header.stamp = ros::Time::now();

        target_marker_.ns = ns;
        target_marker_.id = id;
        target_marker_.type = type;

        target_marker_.action = visualization_msgs::Marker::ADD;

        target_marker_.pose.position.x = p_x;
        target_marker_.pose.position.y = p_y;
        target_marker_.pose.position.z = p_z;
        target_marker_.pose.orientation.x = o_x;
        target_marker_.pose.orientation.y = o_y;
        target_marker_.pose.orientation.z = o_z;
        target_marker_.pose.orientation.w = o_w;

        target_marker_.scale.x = scale;
        target_marker_.scale.y = scale;
        target_marker_.scale.z = scale;

        target_marker_.color.r = colour[0];
        target_marker_.color.g = colour[1];
        target_marker_.color.b = colour[2];
        target_marker_.color.a = colour[3];

        target_marker_.lifetime = ros::Duration();
    }

    void LegKinematics::createLeg()
    {
        for(std::string i :joint_name_)
        {
            ROS_INFO_STREAM("Leg joint:" << i);
        }
    }

    bool LegKinematics::checkKDLChain()
    {
        bool valid = tracik_solver_.getKDLChain(chain_);
        if(!valid) ROS_ERROR("There was no valid KDL chain found");
        return valid;
    }

    std::vector<KDL::JntArray> LegKinematics::getKDLLimits()
    {
        std::vector<KDL::JntArray> limits;
        KDL::JntArray lower_limit, upper_limit;
        bool valid = tracik_solver_.getKDLLimits(lower_limit, upper_limit);
        limits.emplace_back(lower_limit, upper_limit);
        if (!valid)
        {
            ROS_ERROR("There was no valid KDL joint limits found");
        }
        
        return limits;
    }

    KDL::JntArray LegKinematics::getJointsNominal(const KDL::JntArray& lower_limit, const KDL::JntArray& upper_limit)
    {
        KDL::JntArray nominal(number_of_joints_);
        for (size_t i = 0; i < nominal.data.size(); i++)
        {
            nominal(i) = (lower_limit(i) + upper_limit(i)) / 2.0;
            ROS_DEBUG("nominal %li: %f \n", i, nominal(i));
        }
        return nominal;
    }

    auto print_frame_lambda = [](KDL::Frame& f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        ROS_INFO_STREAM(" x:" << x << 
                        " y:" << y << 
                        " z:" << z <<
                        "\n");
        ROS_INFO_STREAM(" roll:" << roll << 
                        " pitch:" << pitch << 
                        " yaw:" << yaw << 
                        "\n");
    };

    KDL::JntArray LegKinematics::inverseKinematics(KDL::JntArray current_joint, KDL::Frame target_frame)
    {
        std_msgs::Float64 joint_msg;
        KDL::JntArray ik_result;

        // joint_state_.header.stamp = ros::Time::now();
        int rc = tracik_solver_.CartToJnt(curr_joint_array_, foot_contact_frame_, ik_result);
        ROS_INFO("ik result status: %i", rc);

        joint_state_.name.resize(number_of_joints_);
        joint_state_.position.resize(number_of_joints_);

        marker_pub_.publish(target_marker_);
        // ROS_DEBUG("Number of joints: %i", number_of_joints_);
        for (size_t i = 0; i < number_of_joints_; i++)
        {
            ROS_INFO("ik_result_ %li  : %f", i, double(ik_result_(i)));
            joint_state_.name[i] = robot_namespace_ + joint_name_[i];
            joint_state_.position[i] = curr_joint_array_.data[i] = ik_result(i);
            joint_msg.data = joint_state_.position[i];
            legPubs_.at(i).publish(joint_msg);
        }
        print_frame_lambda(foot_contact_frame_);

        return ik_result;
    }

    // void LegKinematics::initForwardKinematics()
    // {
    //     KDL::ChainFkSolverPos_recursive fk_solver_(chain_);
    // }

    double LegKinematics::getNumberOfJoints(KDL::Chain& chain)
    {
        double n = chain.getNrOfJoints();
        return n;
    }

    void LegKinematics::forwardKinematics()
    {
        std_msgs::Float64 joint_msg;

        joint_state_.header.stamp = ros::Time::now();
        fk_solver_.JntToCart(joy_joints_, foot_contact_frame_);
        joint_state_.name.resize(number_of_joints_);
        joint_state_.position.resize(number_of_joints_);
        for (size_t i = 0; i < number_of_joints_; i++)
        {
            joint_state_.name[i] = robot_namespace_ + joint_name_[i];
            joint_state_.position[i] = joy_joints_(i);
            joint_msg.data = joint_state_.position[i];
            legPubs_.at(i).publish(joint_msg);
        }   
        print_frame_lambda(foot_contact_frame_);
    }

    void LegKinematics::publishJointStates(const KDL::JntArray& jntarray)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(number_of_joints_);
        joint_state.position.resize(number_of_joints_);

        std_msgs::Float64 joint_msg;

        for (size_t i = 0; i < number_of_joints_; i++)
        {
            joint_state.name[i] = robot_namespace_ + joint_name_[i];
            joint_state.position[i] = curr_joint_array_.data[i] = jntarray(i);

            joint_msg.data = jntarray(i);
            legPubs_.at(i).publish(joint_msg);
        }
    }

    void LegKinematics::updateFSM()
    {
        switch (leg_mode_)
        {
        case Mode::cartesian_mode:
            {
                // ROS_DEBUG("IK mode");
                inverseKinematics();
                break;
            }

        case Mode::joint_mode:
            {
                // ROS_DEBUG("FK mode");
                forwardKinematics();
                break;
            }

        case Mode::bezier_mode:
            {
                ROS_INFO("Bezier mode not implemented yet");
            }
            break;

        case Mode::wheel_mode:
            {
                ROS_INFO("wheel mode");

                // std_msgs::Float64 LBU_J_msg;
                // std_msgs::Float64 LBL_J_msg;
                // std_msgs::Float64 RBU_J_msg;
                // std_msgs::Float64 RBL_J_msg;

                // LBU_J_msg.data = 1.4f;
                // LBL_J_msg.data = 0.0f;
                // RBU_J_msg.data = 1.4f;
                // RBL_J_msg.data = 0.0f;

                // LBU_J_pub.publish(LBU_J_msg);                
                // LBL_J_pub.publish(LBL_J_msg);                
                // RBU_J_pub.publish(RBU_J_msg);                
                // RBL_J_pub.publish(RBL_J_msg);    

                // std_msgs::Float64 LFU_J_msg;
                // std_msgs::Float64 LFL_J_msg;
                // std_msgs::Float64 RFU_J_msg;
                // std_msgs::Float64 RFL_J_msg;

                // LFU_J_msg.data = 1.4f;
                // LFL_J_msg.data = 0.0f;
                // RFU_J_msg.data = 1.4f;
                // RFL_J_msg.data = 0.0f;

                // LFU_J_pub.publish(LFU_J_msg);                
                // LFL_J_pub.publish(LFL_J_msg);                
                // RFU_J_pub.publish(RFU_J_msg);                
                // RFL_J_pub.publish(RFL_J_msg);          
            }
            break;

        default:
            {
                ROS_INFO("default fsm");
            }
            break;
        }
    }

    void LegKinematics::joyCb(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if (msg->buttons[0] == 1)
        {
            leg_mode_ = Mode::cartesian_mode;
        }
        else if (msg->buttons[1] == 1)
        {
            leg_mode_ = Mode::joint_mode;
        }
        else if (msg->buttons[2] == 1)
        {
            leg_mode_ = Mode::bezier_mode;
        } 
        else if (msg->buttons[3] == 1)
        {
            leg_mode_ = Mode::wheel_mode;
        } 
 
        if (leg_mode_ == Mode::cartesian_mode)
        {
            target_marker_.pose.position.x = msg->axes[0] * scale_x_ + bias_x_;
            target_marker_.pose.position.y = msg->axes[3] * scale_y_ + bias_y_;
            target_marker_.pose.position.z = msg->axes[1] * scale_z_ + bias_z_;
            foot_contact_frame_.p.x(target_marker_.pose.position.x);
            foot_contact_frame_.p.y(target_marker_.pose.position.y);
            foot_contact_frame_.p.z(target_marker_.pose.position.z);
            target_marker_.header.stamp = ros::Time::now();
        }
        else if (leg_mode_ == Mode::joint_mode)
        {
            joy_joints_(0) = msg->axes[0];
            joy_joints_(1) = msg->axes[1];
            joy_joints_(2) = msg->axes[2];
            joy_joints_(3) = msg->axes[3];
            joy_joints_(4) = msg->axes[4];
        }
        else if (leg_mode_ == Mode::wheel_mode)
        {
            cmd_vel_msg_.linear.x = msg->axes[1]*10.0;
            cmd_vel_msg_.angular.z = msg->axes[2];
            cmd_vel_pub_.publish(cmd_vel_msg_);

            if(msg->buttons[5] == 1)
            {
                // std_msgs::Float64 LFU_J_msg;
                // std_msgs::Float64 LFL_J_msg;
                // std_msgs::Float64 RFU_J_msg;
                // std_msgs::Float64 RFL_J_msg;

                // LFU_J_msg.data = 1.4f;
                // LFL_J_msg.data = -1.8f;
                // RFU_J_msg.data = 1.4f;
                // RFL_J_msg.data = -1.8f;

                // LFU_J_pub.publish(LFU_J_msg);                
                // LFL_J_pub.publish(LFL_J_msg);                
                // RFU_J_pub.publish(RFU_J_msg);                
                // RFL_J_pub.publish(RFL_J_msg);    

                // LFU_J_msg.data = 0.0f;
                // LFL_J_msg.data = 0.0f;
                // RFU_J_msg.data = 0.0f;
                // RFL_J_msg.data = 0.0f;

                // LFU_J_pub.publish(LFU_J_msg);                
                // LFL_J_pub.publish(LFL_J_msg);                
                // RFU_J_pub.publish(RFU_J_msg);                
                // RFL_J_pub.publish(RFL_J_msg);  


            }
        }
    }

    void LegKinematics::bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {

    }

    void LegKinematics::imuCb(const sensor_msgs::ImuConstPtr& msg)
    {
        
    }

} // ns legKinematics_ns