#include "../../include/DJIMotorCtrlESP.hpp"

/* 速度控制 */
// 3508电机
M3508_P19 MOTOR(1);
void setup()
{
  can_init(8, 18, 100);
}
void loop()
{
  MOTOR.set_speed(50, 10);
}

// 2006电机
M2006_P19 MOTOR(1);
void setup()
{
  can_init(8, 18, 100);
}
void loop()
{
  MOTOR.set_speed(50, 10);
}

// GM6020电机
GM6020 MOTOR(1);
void setup()
{
  can_init(8, 18, 100);
}
void loop()
{
  MOTOR.set_speed(100, 10);
}
