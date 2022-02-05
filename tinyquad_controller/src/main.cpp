#include <AxisController.hpp>

ac_ns::AxisController servo_controller(12, 90, -90);

void setup()
{
}

void loop()
{
  servo_controller.runLoop();
}