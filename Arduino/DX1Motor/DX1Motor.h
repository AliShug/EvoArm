/* Do not modify - auto-generated header file
 * Produced by F:\Documents\GitHub\Project-Unity\Arduino\jinjagen.py

 * DESCRIPTION
 * Interface between an arduino microcontroller and AX12x Dynamixel servo
 * motors.
 */

#ifndef _DX1MOTOR_H
#define _DX1MOTOR_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define DX1MOTOR_PING 0x01
#define DX1MOTOR_READ_DATA 0x02
#define DX1MOTOR_WRITE_DATA 0x03
#define DX1MOTOR_REG_WRITE 0x04
#define DX1MOTOR_ACTION 0x05
#define DX1MOTOR_RESET 0x06
#define DX1MOTOR_SYNC_WRITE 0x83
#define DX1MOTOR_RETURN_NONE 0
#define DX1MOTOR_RETURN_READ 1
#define DX1MOTOR_RETURN_ALL 2
#define DX1MOTOR_BAUD_9600 207
#define DX1MOTOR_BAUD_57600 34
#define DX1MOTOR_BAUD_115200 16
#define DX1MOTOR_BAUD_1MBS 1
#define DX1MOTOR_LED_ENABLE 1
#define DX1MOTOR_LED_DISABLE 0
#define DX1MOTOR_PACKET_BUFFER_SIZE 256
#define DX1MOTOR_TX_DELAY_TIME 0
#define DX1MOTOR_BROADCAST_ID 254
#define DX1MOTOR_RX_TIMEOUT 2000
#define DX1MOTOR_ERR_OK 0
#define DX1MOTOR_ERR_TIMEOUT 1
#define DX1MOTOR_ERR_INPUT_VOLTAGE 2
#define DX1MOTOR_ERR_ANGLE_LIMIT 4
#define DX1MOTOR_ERR_OVERHEAT 8
#define DX1MOTOR_ERR_RANGE_LIMIT 16
#define DX1MOTOR_ERR_PROTOCOL 32
#define DX1MOTOR_ERR_BAD_CHECKSUM 64
#define DX1MOTOR_ERR_OVERLOAD 128
#define DX1MOTOR_ERR_INSTRUCTION 256
#define DX1MOTOR_ERR_CORRUPTION -1

class DX1Motor {
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
    bool waitTimeout(int timeout);
    unsigned char computeChecksum(unsigned char *data_blk_ptr, unsigned short data_blk_size);
    void startPacket(unsigned char instruction, bool broadcast = false);
    void bufferParams(unsigned char *data_block, unsigned short block_length);
    void sendPacket();
    int doReceive(int timeout);

public:
    DX1Motor();
    DX1Motor(unsigned char id);
    ~DX1Motor();
    static void Init(int baud_mode, int pin);
    int read(unsigned char adr, unsigned char len, unsigned char *data);
    int write(unsigned char adr, unsigned char len, unsigned char *data);
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
    int getAlarmFlags(int &err);
    int setAlarmFlags(int alarmflags);
    int getShutdownFlags(int &err);
    int setShutdownFlags(int shutdownflags);
    bool getTorqueEnable(int &err);
    int setTorqueEnable(bool torqueenable);
    int getLED(int &err);
    int setLED(int led);
    int getCWMargin(int &err);
    int setCWMargin(int cwmargin);
    int getCCWMargin(int &err);
    int setCCWMargin(int ccwmargin);
    int getCWSlope(int &err);
    int setCWSlope(int cwslope);
    int getCCWSlope(int &err);
    int setCCWSlope(int ccwslope);
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
    int getLock(int &err);
    int setLock(int lock);
    float getPunch(int &err);
    int setPunch(float punch);
};

#endif