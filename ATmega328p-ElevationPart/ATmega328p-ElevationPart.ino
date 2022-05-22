/**
 * @file ATmega328p-ElevationPart.ino
 * @author BG5ABL (sakura@alleysakura.com)
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
#define PULSE_ENCODER_A 9



int ADXL345 = 0x53;

float X_out, Y_out, Z_out, Angle;

SoftwareSerial RS422 = SoftwareSerial(RS422_RX, RS422_TX);

void initADXL345() {
    Wire.begin();                     // Initiate the Wire library
    Wire.beginTransmission(ADXL345);  // Start communicating with the device
    Wire.write(0x2D);  // Access/ talk to POWER_CTL Register - 0x2D
    Wire.write(8);
    Wire.endTransmission();
}

void readADXL345() {
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);  // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);  // Read 6 registers total, each axis
    // value is stored in 2 registers
    X_out = (Wire.read() | Wire.read() << 8);  // X-axis value
    X_out = X_out / 256;  // For a range of +-2g, we need to divide the raw
    Y_out = (Wire.read() | Wire.read() << 8);  // Y-axis value
    Y_out = Y_out / 256;
    Z_out = (Wire.read() | Wire.read() << 8);  // Z-axis value
    Z_out = Z_out / 256;

    if (X_out > 0) {
        Angle = Y_out * 90;
    } else {
        Angle = 180 - Y_out * 90;
    }
}

void printADXL345() {
    RS422.print("Angle:");
    RS422.println(Angle);
    Serial.print("Xa= ");
    Serial.print(X_out);
    Serial.print("   Ya= ");
    Serial.print(Y_out);
    Serial.print("   Za= ");
    Serial.println(Z_out);
    Serial.print("Angle: ");
    Serial.println(Angle);
}

void setup() {
    Serial.begin(9600);
    RS422.begin(9600);
    initADXL345();
    delay(10);
}

void loop() {
    readADXL345();
    printADXL345();
}