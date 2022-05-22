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

#define RS422_TX 2
#define RS422_RX 3

SoftwareSerial RS422 = SoftwareSerial(RS422_RX, RS422_TX);

void setup() {
    Serial.begin(9600);
    RS422.begin(9600);
}

void loop() {
}
