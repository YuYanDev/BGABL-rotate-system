/**
 * @file ATmega328p-AzimuthPart.ino
 * @author BG5ABL(sakura@alleysakura.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <SoftwareSerial.h>
#include <Wire.h>

/**
 * @brief important!!!!
 * For MAX490 Chip
 * TX - TX
 * RX - RX
 */
#define RS422_TX 2
#define RS422_RX 3

/**
 * @brief For Geared Motor Defined
 * 
 */
#define GEARED_MOTOR_DIRECTION_SWITCH 4
#define GEARED_MOTOR_CW_PWM 5
#define GEARED_MOTOR_CCW_PWM 6
#define GEARED_MOTOR_POWER_SWITCH 7

/**
 * @brief Pulse encoder
 * 
 */

#define PULSE_ENCODER_A 8
#define PULSE_ENCODER_B 9

SoftwareSerial RS422 = SoftwareSerial(RS422_RX, RS422_TX);

void setup() {
    Serial.begin(9600);
    RS422.begin(9600);
}

void loop() {
}
