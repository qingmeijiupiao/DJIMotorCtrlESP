/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-10-27 19:58:54
 */
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
    delay(5000);
    MOTOR.set_location(0);
    delay(5000);
}

