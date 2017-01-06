/* Do not modify - auto-generated header file
 * Produced by jinjagen.py

 * DESCRIPTION
 * Interface between an arduino microcontroller and Dynamixel servo motors.
 */

#ifndef _DX2MOTOR_H
#define _DX2MOTOR_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define DX2MOTOR_PING 0x01
#define DX2MOTOR_READ_DATA 0x02
#define DX2MOTOR_WRITE_DATA 0x03
#define DX2MOTOR_REG_WRITE 0x04
#define DX2MOTOR_ACTION 0x05
#define DX2MOTOR_RESET 0x06
#define DX2MOTOR_REBOOT 0x08
#define DX2MOTOR_STATUS 0x55
#define DX2MOTOR_SYNC_READ 0x82
#define DX2MOTOR_SYNC_WRITE 0x83
#define DX2MOTOR_BULK_READ 0x92
#define DX2MOTOR_BULK_WRITE 0x93
#define DX2MOTOR_RETURN_NONE 0
#define DX2MOTOR_RETURN_READ 1
#define DX2MOTOR_RETURN_ALL 2
#define DX2MOTOR_BAUD_9600 0
#define DX2MOTOR_BAUD_57600 1
#define DX2MOTOR_BAUD_115200 2
#define DX2MOTOR_BAUD_1MBS 3
#define DX2MOTOR_LED_RED 1
#define DX2MOTOR_LED_GREEN 2
#define DX2MOTOR_LED_BLUE 4
#define DX2MOTOR_PACKET_BUFFER_SIZE 2048
#define DX2MOTOR_TX_DELAY_TIME 0
#define DX2MOTOR_BROADCAST_ID 254
#define DX2MOTOR_RX_TIMEOUT 2000
#define DX2MOTOR_ERR_OK 0
#define DX2MOTOR_ERR_TIMEOUT 1
#define DX2MOTOR_ERR_HARDWARE 2
#define DX2MOTOR_ERR_PROTOCOL 4
#define DX2MOTOR_ERR_EXECUTION 8
#define DX2MOTOR_ERR_CRC 16
#define DX2MOTOR_ERR_RANGE 32
#define DX2MOTOR_ERR_OVERLENGTH 64
#define DX2MOTOR_ERR_UNDERLENGTH 128
#define DX2MOTOR_ERR_INSTRUCTION 256
#define DX2MOTOR_ERR_ACCESS 512

class DX2Motor {
private:
    unsigned char _id;
    unsigned char *_packet_data;
    unsigned short _packet_length;
    unsigned short _responseParams;
    unsigned char _responseData[16];
    static int CommPin;
    static long Baud;

    static int convertFromTemp(int val);
    static int convertToTemp(int raw);
    static int convertFromVoltage(float val);
    static float convertToVoltage(int raw);
    static int convertFromDegrees(float val);
    static float convertToDegrees(int raw);
    static int convertFromPercentage(float val);
    static float convertToPercentage(int raw);
    unsigned short computeCRC(unsigned char *data_blk_ptr, unsigned short data_blk_size);
    void startPacket(unsigned char instruction, bool broadcast = false);
    void bufferParams(unsigned char *data_block, unsigned short block_length);
    void sendPacket();
    int doReceive();

public:
    DX2Motor();
    DX2Motor(unsigned char id);
    ~DX2Motor();
    static void Init(int baud_mode, int pin);
    int read(unsigned short adr, unsigned short len, unsigned char *data);
    int write(unsigned short adr, unsigned short len, unsigned char *data);
    void retarget(unsigned char id);

    int getModelNumber(int &err);
    int getFirmwareVersion(int &err);
    int getID(int &err);
    int setID(int id);
    int getBaudRate(int &err);
    int setBaudRate(int baudrate);
    int getReturnDelay(int &err);
    int setReturnDelay(int returndelay);
    float getCWLimit(int &err);
    int setCWLimit(float cwlimit);
    float getCCWLimit(int &err);
    int setCCWLimit(float ccwlimit);
    int getControlMode(int &err);
    int setControlMode(int controlmode);
    int getTempLimit(int &err);
    int setTempLimit(int templimit);
    float getLowVoltageLimit(int &err);
    int setLowVoltageLimit(float lowvoltagelimit);
    float getHighVoltageLimit(int &err);
    int setHighVoltageLimit(float highvoltagelimit);
    float getMaxTorque(int &err);
    int setMaxTorque(float maxtorque);
    int getReturnLevel(int &err);
    int setReturnLevel(int returnlevel);
    int getShutdownFlags(int &err);
    int setShutdownFlags(int shutdownflags);
    bool getTorqueEnable(int &err);
    int setTorqueEnable(bool torqueenable);
    int getLED(int &err);
    int setLED(int led);
    int getDGain(int &err);
    int setDGain(int dgain);
    int getIGain(int &err);
    int setIGain(int igain);
    int getPGain(int &err);
    int setPGain(int pgain);
    float getGoalPosition(int &err);
    int setGoalPosition(float goalposition);
    float getGoalSpeed(int &err);
    int setGoalSpeed(float goalspeed);
    float getTorqueLimit(int &err);
    int setTorqueLimit(float torquelimit);
    float getPosition(int &err);
    float getSpeed(int &err);
    float getLoad(int &err);
    float getVoltage(int &err);
    int getTemperature(int &err);
    bool getInstructionStored(int &err);
    bool getMoving(int &err);
    int getHardwareError(int &err);
    float getPunch(int &err);
    int setPunch(float punch);
};

#endif