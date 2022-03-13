#ifndef TINYQUAD_PLANNER_LEG_KINEMATICS_
#define TINYQUAD_PLANNER_LEG_KINEMATICS_
namespace legKinematics_ns
{
    class LegKinematics
    {
    public:
        LegKinematics();
        ~LegKinematics();

    private:
        // limb's limitation
        const double upper_limb_l_ = 120.0f, lower_limb_l_ = 120.0f;
        const double theta_hip_max_ = 45.0f, theta_shoulder_max_ = 45.0f, theta_elbow_max_ = 45.0f;
        const double theta_hip_min_ = -45.0f, theta_shoulder_min_ = -45.0f, theta_elbow_min_ = -45.0f;
        const double theta_hip_home = 0.0f, theta_shoulder_home_ = 0.0f, theta_elbow_home_ = 0.0f;
        static uint32_t upper_limb, lower_limb;
        uint32_t pos_1, pos_2;
        double cartesionCoor_[3] = {0.0f, 0.0f, 0.0f};
        static double theta_hip_curr_, theta_shoulder_curr_, theta_elbow_curr_;
        int num_of_servo_ = 1;

        void inverseKinematics();
        void forwardKinematics();
        void pathSegmentation();

    };
} // ns legKinematics_ns

#endif