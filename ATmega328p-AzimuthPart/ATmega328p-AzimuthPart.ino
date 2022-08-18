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
#include <Wire.h>
// #include <avr/wdt.h>

#define IS_DEBUG true

#define GEARED_MOTOR_POWER_SWITCH 2
#define GEARED_MOTOR_POWER_CW 5
#define GEARED_MOTOR_POWER_CCW 6

int ADXL345 = 0x53;

float ADXL345XOut, ADXL345YOut, ADXL345ZOut, ADXL345ZAngle;

void startRotateCW(String serialString) {
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 0);
    digitalWrite(GEARED_MOTOR_POWER_CCW, 0);
    digitalWrite(GEARED_MOTOR_POWER_CW, 1);
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 1);
    Serial.println("+CW:OK");
}

void startRotateCCW(String serialString) {
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 0);
    digitalWrite(GEARED_MOTOR_POWER_CW, 0);
    digitalWrite(GEARED_MOTOR_POWER_CCW, 1);
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 1);
    Serial.println("+CCW:OK");
}

void stopRotate() {
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 0);
    digitalWrite(GEARED_MOTOR_POWER_CW, 0);
    digitalWrite(GEARED_MOTOR_POWER_CCW, 0);
    Serial.println("+STOP:OK");
}

void resetRotate() {}

void readADXL345() {
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32);
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true);

    ADXL345XOut = (Wire.read() | Wire.read() << 8);
    ADXL345XOut = ADXL345XOut / 256;
    ADXL345YOut = (Wire.read() | Wire.read() << 8);
    ADXL345YOut = ADXL345YOut / 256;
    ADXL345ZOut = (Wire.read() | Wire.read() << 8);
    ADXL345ZOut = ADXL345ZOut / 256;

    if (ADXL345XOut > 0) {
        ADXL345ZAngle = ADXL345YOut * 90;
    } else {
        ADXL345ZAngle = 180 - ADXL345YOut * 90;
    }
}

void printADXL345() {
    Serial.print("+ANGLE:");
    Serial.print(ADXL345XOut);
    Serial.print(",");
    Serial.print(ADXL345YOut);
    Serial.print(",");
    Serial.print(ADXL345ZOut);
    Serial.print(",");
    Serial.println(ADXL345ZAngle);
}

void onSerialCall() {
    String serialString;

    // 当串口有数据则循环运行如下处理
    while (Serial.available()) {
        serialString = serialString + (char)Serial.read();
    }

    if (IS_DEBUG) {
        Serial.print("GetSerial");
        Serial.println(serialString);
    }

    if (serialString == "AT+HAND") {
        Serial.print("+HAND:OK");
        // wdt_enable(WDTO_2S);
        return;
    }

    if (serialString == "AT+BYE") {
        Serial.print("+BYE:OK");
        // wdt_disable();
    }

    if (serialString.indexOf("AT+CCW") > -1) {
        startRotateCCW(serialString);
        // wdt_reset();
        return;
    }

    if (serialString.indexOf("AT+CW") > -1) {
        startRotateCW(serialString);
        // wdt_reset();
        return;
    }

    if (serialString == "AT+STOP") {
        stopRotate();
        // wdt_reset();
        return;
    }

    if (serialString == "AT+RESET") {
        resetRotate();
        // wdt_reset();
        return;
    }

    if (serialString == "AT+ANGLE?") {
        readADXL345();
        printADXL345();
        // wdt_reset();
        return;
    }
}

void initPIN() {
    pinMode(GEARED_MOTOR_POWER_SWITCH, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 0);
    pinMode(GEARED_MOTOR_POWER_CW, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_CW, 0);
    pinMode(GEARED_MOTOR_POWER_CCW, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_CCW, 0);
}

void initADXL345() {
    Wire.begin();
    Wire.beginTransmission(ADXL345);
    Wire.write(0x2D);
    Wire.write(8);
    Wire.endTransmission();
}

void setup() {
    Serial.begin(9600);
    initPIN();
    initADXL345();
    Serial.println("+SYS:OK");
    delay(100);
}

void loop() {
    if (Serial.available()) {
        onSerialCall();
    }
    delay(100);
}