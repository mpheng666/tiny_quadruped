#include <Arduino.h>
#include <legKinematics.hpp>

#define DEBUG_MODE 1

#if DEBUG_MODE == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

legKinematics_ns::LegKinematics front_left;
auto servo = front_left.startServo();
const byte button_pin = 20;
bool servo_enable = true;
long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 200;   // the debounce time; increase if the output flickers

void button_control()
{
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (servo_enable == true)
    {
      servo_enable = false;
    }
    else if (servo_enable == false)
    {
      servo_enable = true;
    }
    lastDebounceTime = millis();
    Serial.printf("Servo enable state: %i \n", servo_enable);
  }
}

void setup()
{
  while (!Serial)
    ;
  Serial.begin(115200);
  debugln("Started serial communication!");
  pinMode(button_pin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(button_pin), button_control, RISING);
}

void loop()
{
  if (servo_enable)
  {
    front_left.executeServo(servo);
  }
}