/*
 Name:		Comms.ino
 Created:	4/26/2016 4:55:32 PM
 Author:	alist
*/

#include <CapacitiveSensor.h>

#include "Protocol.h"
#include <DX1Motor.h>
#include <DX2Motor.h>


#define BUFFER_LEN 1024

bool waitTimeout(Stream &s, unsigned long wait) {
    unsigned long start = millis();

    do {
        if (s.peek() >= 0) {
            return true;
        }
    } while (millis() - start < wait);

    return false;
}

DX1Motor x1Servos[10];
DX2Motor x2Servos[10];
int nx1 = 0;
int nx2 = 0;

// Init - Find servos
void setup() {
    pinMode(2, OUTPUT);
    digitalWrite(2, 0);

    DX1Motor::Init(DX1MOTOR_BAUD_1MBS, 2);
    DX2Motor::Init(DX2MOTOR_BAUD_1MBS, 2);
    Serial.begin(250000);
    while (!Serial);

    // Fill out our servo listings
    // LED blinks at start and finish
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    delay(100);
    digitalWrite(13, HIGH);

    for (int checkid = 0; checkid < 252; checkid++) {
        DX1Motor servo(checkid);
        int err;

        if (servo.getID(err), err == DX1MOTOR_ERR_OK) {
            x1Servos[nx1++] = servo;
            /*servo.setCWLimit(0.0f);
            servo.setCCWLimit(300.0f);
            delay(10);*/
        }
    }
    for (int checkid = 0; checkid < 252; checkid++) {
        DX2Motor servo(checkid);
        int err;

        if (servo.getID(err), err == DX2MOTOR_ERR_OK) {
            x2Servos[nx2++] = servo;
            servo.setCWLimit(0.0f);
            servo.setCCWLimit(300.0f);
            servo.setControlMode(2);
        }
    }

    // Low latency
    for (int i = 0; i < nx1; i++) {
        x1Servos[i].setReturnLevel(1);
        x1Servos[i].setReturnDelay(0);
    }
    for (int i = 0; i < nx2; i++) {
        x2Servos[i].setReturnDelay(0);
    }
    
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);

    CapacitiveSensor capSense(30, 31);

    Serial.println("CommTest READY");

    Protocol protocol(x1Servos, nx1, x2Servos, nx2);
    while (true) {
        // Execute protocol
        protocol.handleIncoming(Serial);
    }
}

void loop() {}
