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
#define PULSE_ENCODER_B 9

int ADXL345 = 0x53;

float ADXL345XOut, ADXL345YOut, ADXL345ZOut, ADXL345ZAngle;

int PreviousPulseEncoderStatusA = 1;
int PreviousPulseEncoderStatusB = 0;
int PulseEncoderCount = 0;

int GearedMotorStatus = 0;

SoftwareSerial RS422 = SoftwareSerial(RS422_RX, RS422_TX);

char buffer1[24];

void initADXL345() {
    Wire.begin();                     // Initiate the Wire library
    Wire.beginTransmission(ADXL345);  // Start communicating with the device
    Wire.write(0x2D);  // Access/ talk to POWER_CTL Register - 0x2D
    Wire.write(8);
    Wire.endTransmission();
}

void initPulseEncoder() {
    pinMode(PULSE_ENCODER_A, INPUT);
    pinMode(PULSE_ENCODER_B, INPUT);
    if (digitalRead(PULSE_ENCODER_A) != digitalRead(PULSE_ENCODER_B)) {
        PreviousPulseEncoderStatusA = digitalRead(PULSE_ENCODER_A);
        PreviousPulseEncoderStatusB = digitalRead(PULSE_ENCODER_B);
    } else {
        Serial.println("ERROR:PULSE_ENCODER_FAIL");
    }
}

void readADXL345() {
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);  // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);  // Read 6 registers total, each axis
    // value is stored in 2 registers
    ADXL345XOut = (Wire.read() | Wire.read() << 8);  // X-axis value
    ADXL345XOut =
        ADXL345XOut / 256;  // For a range of +-2g, we need to divide the raw
    ADXL345YOut = (Wire.read() | Wire.read() << 8);  // Y-axis value
    ADXL345YOut = ADXL345YOut / 256;
    ADXL345ZOut = (Wire.read() | Wire.read() << 8);  // Z-axis value
    ADXL345ZOut = ADXL345ZOut / 256;

    if (ADXL345XOut > 0) {
        ADXL345ZAngle = ADXL345YOut * 90;
    } else {
        ADXL345ZAngle = 180 - ADXL345YOut * 90;
    }
}

void readPulseEncoder() {
    const int currentPulseEncoderStatusA = digitalRead(PULSE_ENCODER_A);
    const int currentPulseEncoderStatusB = digitalRead(PULSE_ENCODER_B);
    if (currentPulseEncoderStatusA != currentPulseEncoderStatusB) {
        if (PreviousPulseEncoderStatusA != currentPulseEncoderStatusA &&
            PreviousPulseEncoderStatusB != currentPulseEncoderStatusB) {
            PreviousPulseEncoderStatusA = currentPulseEncoderStatusA;
            PreviousPulseEncoderStatusB = currentPulseEncoderStatusB;
            PulseEncoderCount++;
        }
    } else {
        Serial.println("ERROR:PULSE_ENCODER_FAIL");
    }
}

void printADXL345() {
    RS422.print("ADXL345Z_ANGLE:");
    RS422.println(ADXL345ZAngle);
    // Serial.print("Xa= ");
    // Serial.print(ADXL345XOut);
    // Serial.print("   Ya= ");
    // Serial.print(ADXL345YOut);
    // Serial.print("   Za= ");
    // Serial.println(ADXL345ZOut);
    // Serial.print("ADXL345ZAngle: ");
    // Serial.println(ADXL345ZAngle);
}

void setup() {
    Serial.begin(9600);
    RS422.begin(9600);
    initADXL345();
    delay(10);
    // initPulseEncoder();
}

void loop() {
    readADXL345();
    // readPulseEncoder();
    printADXL345();
    if (RS422.available() > 0) {
        delay(100);
        RS422.readBytes(buffer1, 12);
        Serial.println(buffer1);
    }
}
