#include "DJIMotorCtrlESP.hpp"


// 请确保你开发板连接了can收发芯片，并且总线上至少有一个120欧姆电阻
//Please ensure that your development board is connected to the CAN transceiver chip 
//there is at least one 120 ohm resistor on the bus


// 创建一个 M3508_P19 类的对象 MOTOR，并将其 ID 设置为 1
// Create an object MOTOR of the M3508_P19 class, and set its ID to 1
M3508_P19 MOTOR(1);

void setup() {

  // 初始化 CAN 总线,在使用电机之前必须调用 can_init 函数
  // Initialize the CAN bus,before use motor, must call can_init first
  // 参数 8: can收发芯片 TX 引脚
  // Parameter 8: transceiver chip TX pin
  // 参数 18: can收发芯片 RX 引脚
  // Parameter 18: transceiver chip RX pin
  // 参数 100: 当前电流更新频率，单位为赫兹
  // Parameter 100: Current update frequency in Hz
  can_init(8, 18, 100); //

  while(MOTOR.is_online()==false){
    // 等待电机上线
    // Wait for the motor to be online
    delay(100);
  }
}

void loop() {
  // 正向加速循环
  // Forward acceleration loop
  for (int i = 1; i < 30; i++) {
    // 设置电机的速度
    // Set the speed of the motor
    MOTOR.set_speed(i * 10);
    // 延迟 100 毫秒，以便可以观察到速度的变化
    // Delay 100 milliseconds to observe the change in speed
    delay(100);
  }
  // 在达到最高速度后，延迟 1500 毫秒
  // Delay 1500 milliseconds after reaching the highest speed
  delay(1500);

  // 反向加速循环
  // Reverse acceleration loop
  for (int i = 1; i < 30; i++) {
    // 设置电机的速度
    // Set the speed of the motor
    MOTOR.set_speed(i * -10);
    // 延迟 100 毫秒，以便可以观察到速度的变化
    // Delay 100 milliseconds to observe the change in speed
    delay(100);
  }
  // 在达到最高反向速度后，延迟 1500 毫秒
  // Delay 1500 milliseconds after reaching the highest reverse speed
  delay(1500);
}
