#include <legKinematics.hpp>

namespace legKinematics_ns
{
    LegKinematics::LegKinematics() : pos_1(SERVOMIN), pos_2(SERVOMAX)
    {
        // constructor
        Serial.print("Constructor initiated");
    }
    LegKinematics::~LegKinematics()
    {
        // destructor
    }

    Adafruit_PWMServoDriver LegKinematics::startServo()
    {
        // define PWMServoDriver based on the number of legs
        Adafruit_PWMServoDriver hip_1 = Adafruit_PWMServoDriver();
        hip_1.begin();
        hip_1.setPWMFreq(50);

        return hip_1;
    }

    void LegKinematics::executeServo(Adafruit_PWMServoDriver &servo)
    {
        for (int i = 0; i < num_of_servo_; i++)
        {
            for (int angle = 0; angle < 180; angle += 1)
            {
                auto pwm = this->angleToPulse(angle);
                Serial.printf("moving servo %d \n", pwm);
                servo.setPWM(i, 0, pwm);
                delay(10); // wait for 1 second
            }
            for (int angle = 180; angle > 0; angle -= 1)
            {
                auto pwm = this->angleToPulse(angle);
                Serial.printf("moving servo %d \n", pwm);
                servo.setPWM(i, 0, pwm);
                delay(10); // wait for 1 second
            }
        }
            // Serial.printf("moving servo to %i \t", pos_1);
            // delay(1000); // wait for 1 second
            // Serial.printf("moving servo to %i \n", pos_2);
            // delay(1000); // wait for 1 second
    }

    int LegKinematics::angleToPulse(int ang)
    {
        int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
        return pulse;
    }

    void LegKinematics::inverseKinematics()
    {
        // given XYZ, calculate thetas
    }
    void LegKinematics::forwardKinematics()
    {
        // given thetas, calculate XYZ
    }

} // ns legKinematics_ns