// Protocol.h

#ifndef _PROTOCOL_h
#define _PROTOCOL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <DX1Motor.h>
#include <DX2Motor.h>
#include <CapacitiveSensor.h>

class Protocol {
private:
    bool waitTimeout(Stream &s, long time);
    void dispatchV1Command(Stream &s, int mode, char command, char id);
    void dispatchV2Command(Stream &s, int mode, char command, char id);

    union {
        float _argf;
        long _argi;
    };

    DX1Motor *_x1s;
    DX2Motor *_x2s;
    CapacitiveSensor _capSense;
    int _nx1, _nx2;

    const static int MODE_GET = 0, MODE_SET = 1;

public:
    Protocol(DX1Motor *x1s, int nx1, DX2Motor *x2s, int nx2);
    void handleIncoming(Stream &s);
};

#endif

