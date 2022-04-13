#include "legKinematics.hpp"

namespace legKinematics_ns
{
    LegKinematics::LegKinematics(ros::NodeHandle& nh, const std::string prefix, const int x_mirror, const int y_mirror) :
    prefix(prefix),
    x_mirror(x_mirror),
    y_mirror(y_mirror),
    // spinner(4),
    joint_pub(p_nh_.advertise<sensor_msgs::JointState>("joint_states", 1)),
    marker_pub(p_nh_.advertise<visualization_msgs::Marker>("visualization_target_marker", 1)),
    bezier_sub(p_nh_.subscribe<geometry_msgs::PointStamped>("bezier_points", 100, &LegKinematics::bezierCb, this)),
    joy_sub(p_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &LegKinematics::joyCb, this)),
    tracik_solver(chain_start, chain_end, urdf_param, timeout, eps),
    fk_solver(chain)
    {

    }
    LegKinematics::~LegKinematics()
    {
    }

    void LegKinematics::start()
    {
        ros::Rate r(LOOP_RATE);
        this->loadParam();
        this->createLeg();
        this->initTf();
        this->initMarker();
        
        if(this->checkKDLChain()) 
        {
            this->getKDLLimits();
            this->getJointsNominal();
            this->initForwardKinematics();
        }

        while(ros::ok())
        {
            this->updateFSM();
            joint_pub.publish(joint_state);
            ros::spinOnce();
            r.sleep();
        }
    }

    void LegKinematics::loadParam()
    {

    }

    void LegKinematics::initTf()
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "foot_contact_frame";
        tf.setOrigin( tf2::Vector3(0.0, 0.0, 0.0) );
        q.setRPY(0.0, 0.0, 0.0);
        tf.setRotation(q);
        tf_bc.sendTransform(transformStamped);
    }

    void LegKinematics::initMarker()
    {
        target_marker.header.frame_id = "foot_contact_frame";
        target_marker.header.stamp = ros::Time::now();

        target_marker.ns = "foot";
        target_marker.id = 0;
        target_marker.type = shape;

        target_marker.action = visualization_msgs::Marker::ADD;

        target_marker.pose.position.x = 0;
        target_marker.pose.position.y = 0;
        target_marker.pose.position.z = 0;
        target_marker.pose.orientation.x = 0.0;
        target_marker.pose.orientation.y = 0.0;
        target_marker.pose.orientation.z = 0.0;
        target_marker.pose.orientation.w = 1.0;

        target_marker.scale.x = 0.025;
        target_marker.scale.y = 0.025;
        target_marker.scale.z = 0.025;

        target_marker.color.r = 0.0f;
        target_marker.color.g = 1.0f;
        target_marker.color.b = 0.0f;
        target_marker.color.a = 1.0;

        target_marker.lifetime = ros::Duration();
    }

    void LegKinematics::createLeg()
    {
        for(std::string i :joint_name)
        {
            ROS_INFO_STREAM("Leg joint:" << i);
        }
    }

    bool LegKinematics::checkKDLChain()
    {
        bool valid = tracik_solver.getKDLChain(chain);
        if(!valid) ROS_ERROR("There was no valid KDL chain found");
        return valid;
    }

    void LegKinematics::getKDLLimits()
    {
        bool valid = tracik_solver.getKDLLimits(lower_limit, upper_limit);
        if (!valid)
        {
            ROS_ERROR("There was no valid KDL joint limits found");
        }
    }

    void LegKinematics::getJointsNominal()
    {
        KDL::JntArray nominal(number_of_joints);
        for (size_t i = 0; i < nominal.data.size(); i++)
        {
            nominal(i) = (lower_limit(i) + upper_limit(i)) / 2.0;
            printf("nominal %li: %f \n", i, nominal(i));
        }
        curr_joint_array = nominal;
    }

    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };

    void LegKinematics::inverseKinematics()
    {
        joint_state.header.stamp = ros::Time::now();
        int rc = tracik_solver.CartToJnt(curr_joint_array, foot_contact_frame, ik_result);
        ROS_INFO("ik result status: %i", rc);
        joint_state.name.resize(number_of_joints);
        joint_state.position.resize(number_of_joints);

        marker_pub.publish(target_marker);
        for (size_t i = 0; i < number_of_joints; i++)
        {
            ROS_INFO("ik_result: %f", double(ik_result(i)));
            joint_state.name[i] = joint_name[i];
            joint_state.position[i] = curr_joint_array.data[i] = ik_result(i);
        }
        print_frame_lambda(foot_contact_frame);
    }

    void LegKinematics::initForwardKinematics()
    {
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        number_of_joints = chain.getNrOfJoints();
        ROS_INFO("joints number: %d", number_of_joints);
    }

    void LegKinematics::forwardKinematics()
    {
        joint_state.header.stamp = ros::Time::now();
        fk_solver.JntToCart(joy_joints, foot_contact_frame);
        joint_state.name.resize(number_of_joints);
        joint_state.position.resize(number_of_joints);
        for (size_t i = 0; i < number_of_joints; i++)
        {
            joint_state.name[i] = joint_name[i];
            joint_state.position[i] = joy_joints(i);
        }   
        print_frame_lambda(foot_contact_frame);
    }

    void LegKinematics::updateFSM()
    {
        switch (leg_mode)
        {
        case Mode::cartesian_mode:
            {
                inverseKinematics();
                break;
            }

        case Mode::joint_mode:
            {
                forwardKinematics();
                break;
            }

        case Mode::bezier_mode:
            {
                ROS_INFO("Bezier mode not implemented yet");
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
            leg_mode = Mode::cartesian_mode;
        }
        else if (msg->buttons[1] == 1)
        {
            leg_mode = Mode::joint_mode;
        }
        else if (msg->buttons[2] == 1)
        {
            leg_mode = Mode::bezier_mode;
        }

        if (leg_mode == Mode::cartesian_mode)
        {
            target_marker.pose.position.x = msg->axes[0]*scale_x + bias_x;
            target_marker.pose.position.y = msg->axes[3]*scale_y + bias_y;
            target_marker.pose.position.z = msg->axes[1]*scale_z + bias_z;
            foot_contact_frame.p.x(target_marker.pose.position.x);
            foot_contact_frame.p.y(target_marker.pose.position.y);
            foot_contact_frame.p.z(target_marker.pose.position.z);
            target_marker.header.stamp = ros::Time::now();
        }
        else if (leg_mode == Mode::joint_mode)
        {
            joy_joints(0) = msg->axes[0];
            joy_joints(1) = msg->axes[1];
            joy_joints(2) = msg->axes[2];
            joy_joints(3) = msg->axes[3];
        }
    }

    void LegKinematics::bezierCb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {

    }

} // ns legKinematics_ns