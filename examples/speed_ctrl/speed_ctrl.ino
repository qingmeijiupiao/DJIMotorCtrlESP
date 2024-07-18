#include "DJIMotorCtrlESP.hpp"
M2006_P36 MOTOR(1);
void setup() {
  can_init(8, 18, 100);

}
void loop() {
  MOTOR.set_location(8192*12);
}
