#include "../../include/DJIMotorCtrlESP.hpp"

/* 位置控制 */

// 3508电机
M3508_P19 MOTOR(1);
void setup()
{
    can_init(8, 18, 100);
    MOTOR.setup();
}
void loop()
{
    MOTOR.set_location(8192 * 20);
}

// 2006电机
M2006_P36 MOTOR(1);
void setup()
{
    can_init(8, 18, 100);
    MOTOR.setup();
}
void loop()
{
    MOTOR.set_location(8192 * 20);
}

// GM6020电机
GM6020 MOTOR(1);
void setup()
{
    can_init(8, 18, 100);
    MOTOR.setup();
}
void loop()
{
    MOTOR.set_location(8192 * 20);
}