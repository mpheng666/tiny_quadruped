#include <AxisController.hpp>

namespace ac_ns
{
    /**
     * @brief Construct a new Axis Controller:: Axis Controller object
     *
     * @param num_of_servo
     * @param angle_max
     * @param angle_min
     */
    AxisController::AxisController(int num_of_servo, int16_t angle_max, int16_t angle_min)
        : enable_button(BUTTON_PIN), curr_servo_enable_(false)
    {
        Serial.begin(115200);
        Serial.println("Constructor initiated");
        begin();
    }

    /**
     * @brief Destroy the Axis Controller:: Axis Controller object
     *
     */
    AxisController::~AxisController()
    {
    }

    /**
     * @brief initialise PCA9685 and set pwm frequency followed by set home
     *
     */
    void AxisController::begin()
    {
        axis.begin();
        axis.setPWMFreq(SERVO_FREQ);
        this->setHome();
    }

    /**
     * @brief set all servo to defined home
     *
     */
    void AxisController::setHome()
    {
        this->executeServos(home_angle_);
    }

    void AxisController::runLoop()
    {
        if (this->checkButton())
        {
            this->executeServos(this->curr_joint_angle_);
        }
        else
        {
            this->executeServos(this->test_joint_angle_);
        }
    }

    bool AxisController::checkButton()
    {
        // Serial.printf("state: %i \t", enable_button.getState());
        // Serial.printf("button: %i \n", enable_button.isPressed());

        if (enable_button.isPressed())
        {
            if (curr_servo_enable_ == true)
            {
                curr_servo_enable_ = false;
            }
            else
            {
                curr_servo_enable_ = true;
            }
        }
        Serial.printf("curr_servo_enable_: %i \n", curr_servo_enable_);
        return curr_servo_enable_;
    }

    /**
     * @brief execute specific servo incrementally from start to end
     *
     * @param servo_index
     * @param start_pos
     * @param end_pos
     * @param increment
     */
    void AxisController::executeServo(int servo_index, int16_t start_pos, int16_t end_pos, int16_t increment)
    {
        unsigned long prev_time = 0;
        unsigned long interval = 2000;
        {
            for (int angle = start_pos; angle != end_pos; angle += increment)
            {
                auto pwm = this->angleToPulse(angle);
                if (millis() - prev_time > interval)
                {
                    axis.setPWM(servo_index, 0, pwm);
                    prev_time = millis();
                }
            }
        }
    }

    /**
     * @brief  map angle of 0 to 180 to Servo min and Servo max
     *
     * @param angle
     * @return pwm pulse
     */
    int AxisController::angleToPulse(int &angle)
    {
        int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
        return pulse;
    }

    /**
     * @brief execute specific servo with target angle
     *
     * @param servo_index
     * @param target_angle
     */
    void AxisController::executeTargetAngle(int servo_index, int target_angle)
    {
        auto pwm = this->angleToPulse(target_angle);
        axis.setPWM(servo_index, 0, pwm);
    }

    /**
     * @brief execute all servo with target angles
     *
     * @param joint_array
     */
    void AxisController::executeServos(std::vector<int16_t> joint_array)
    {
        for (uint16_t i = 0; i < joint_array.size(); i++)
        {
            this->executeTargetAngle(i, joint_array[i]);
        }
    }

    void AxisController::jointParser(int16_t *new_angle_)
    {
        for (uint16_t i = 0; i < sizeof(curr_joint_angle_); i++)
        {
            curr_joint_angle_[i] = new_angle_[i];
        }
    }

} // ac_ns