#ifndef AXIS_CONTROLLER_
#define AXIS_CONTROLLER_

#define DEBUG_MODE 1

#if DEBUG_MODE == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x) Serial.printf(x)
#else
#define debug(x)
#define debugln(x)
#define debugf(x)
#endif

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Button.hpp>

#include <list>
#include <deque>
#include <queue>
#include <vector>
#include <iterator>

#define SERVOMIN 145
#define SERVOMAX 550
#define SERVO_FREQ 50
#define BUTTON_PIN 20

typedef enum quadruped_axis
{
    front_left_shoulder = 0,
    front_left_quad,
    front_left_shin,
    front_right_shoulder,
    front_right_quad,
    front_right_shin,
    back_left_shoulder,
    back_left_quad,
    back_left_shin,
    back_right_shoulder,
    back_right_quad,
    back_right_shin,
} QuadrupedAxis;

namespace ac_ns
{
    class AxisController
    {
    public:
        AxisController(int, int16_t, int16_t);
        virtual ~AxisController();
        void runLoop();

    private:
        // Class object declaration
        Adafruit_PWMServoDriver axis;
        button_ns::Button enable_button;

        // servo related variables
        bool curr_servo_enable_;
        bool prev_servo_enable_;
        int16_t angle_max, angle_min;
        std::vector<int16_t> curr_joint_angle_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<int16_t> prev_joint_angle_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<int16_t> test_joint_angle_ = {90, 180, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<int16_t> home_angle_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        // Serial reading
        std::vector<int8_t> serial_buffer_;

        // PID related variables
        std::vector<int16_t> error_joint_angle;
        int16_t P_value;
        int16_t I_value;
        int16_t D_value;

        uint8_t receiveSerialCommand();
        uint8_t sendSerialCommand();
        void decodeSerialCommand();
        bool checkButton();

        void begin();
        void setHome();
        void PIDControl();
        int angleToPulse(int &ang);
        void executeTargetAngle(int, int);
        void executeServo(int, int16_t, int16_t, int16_t);
        void executeServos(std::vector<int16_t>);
        void jointParser(int16_t *);

    };
} // ac_ns

#endif