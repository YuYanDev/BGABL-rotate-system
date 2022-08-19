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

// #include <avr/wdt.h>

#define IS_DEBUG true

#define GEARED_MOTOR_POWER_SWITCH 2
#define GEARED_MOTOR_POWER_CW 5
#define GEARED_MOTOR_POWER_CCW 6


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
}

void initPIN() {
    pinMode(GEARED_MOTOR_POWER_SWITCH, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_SWITCH, 0);
    pinMode(GEARED_MOTOR_POWER_CW, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_CW, 0);
    pinMode(GEARED_MOTOR_POWER_CCW, OUTPUT);
    digitalWrite(GEARED_MOTOR_POWER_CCW, 0);
}


void setup() {
    Serial.begin(9600);
    initPIN();
    Serial.println("+SYS:OK");
    delay(100);
}

void loop() {
    if (Serial.available()) {
        onSerialCall();
    }
    delay(100);
}