/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-08-13 06:38:53
 */
#ifndef ARDUINO_H
#define ARDUINO_H
#ifdef __cplusplus
extern "C" {
#endif


#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#define PI         3.1415926535897932384626433832795
#define HALF_PI    1.5707963267948966192313216916398
#define TWO_PI     6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER      2.718281828459045235360287471352

#define _min(a, b)                ((a) < (b) ? (a) : (b))
#define _max(a, b)                ((a) > (b) ? (a) : (b))
#define _abs(x)                   ((x) > 0 ? (x) : -(x))  // abs() comes from STL
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _round(x)                 ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))  // round() comes from STL
#define radians(deg)              ((deg) * DEG_TO_RAD)
#define degrees(rad)              ((rad) * RAD_TO_DEG)
#define sq(x)                     ((x) * (x))

#define sei()          portENABLE_INTERRUPTS()
#define cli()          portDISABLE_INTERRUPTS()
#define interrupts()   sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond()  ((long int)getCpuFrequencyMhz())
#define clockCyclesToMicroseconds(a) ((a) / clockCyclesPerMicrosecond())
#define microsecondsToClockCycles(a) ((a) * clockCyclesPerMicrosecond())

#define lowByte(w)  ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit)          ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

void delay(int);
int64_t millis();
int64_t micros();



#ifdef __cplusplus
    }
#endif
#endif