/* Do not modify - auto-generated code file
 * Produced by F:\Documents\GitHub\Project-Unity\Arduino\jinjagen.py

 * DESCRIPTION
 * Interface between an arduino microcontroller and AX12x Dynamixel servo
 * motors.
 */

#include "DX1Motor.h"

#define TemplateSerial Serial1

int DX1Motor::CommPin;
long DX1Motor::Baud;

//// INCLUDED FROM dx1_codefile.cpp

bool DX1Motor::waitTimeout(int timeout) {
    unsigned long startTime = micros();

    do {
        if (TemplateSerial.peek() >= 0) {
            return true;
        }
    } while (micros() - startTime < timeout);

    return false;
}

// Must be called before any instance use
void DX1Motor::Init(int baud_mode, int pin) {
    CommPin = pin;

    switch (baud_mode) {
    case DX1MOTOR_BAUD_9600:
        Baud = 9600;
        break;
    case DX1MOTOR_BAUD_57600:
        Baud = 57600;
        break;
    case DX1MOTOR_BAUD_115200:
        Baud = 115200;
        break;
    case DX1MOTOR_BAUD_1MBS:
    default:
        Baud = 1000000;
        break;
    }


    pinMode(CommPin, OUTPUT);
    digitalWrite(CommPin, 1);
    TemplateSerial.begin(Baud);
    while (!TemplateSerial) ;
    TemplateSerial.setTimeout(1);
}

void DX1Motor::retarget(unsigned char id) {
    _id = id;
}

DX1Motor::DX1Motor(unsigned char id) {
    _id = id;
    _packet_data = NULL;
}

DX1Motor::DX1Motor() {
    _id = 0;
    _packet_data = NULL;
}


DX1Motor::~DX1Motor() {
    if (_packet_data != NULL) {
        delete _packet_data;
        _packet_data = NULL;
    }
}

// Unit Conversions
int DX1Motor::convertFromTemp(int val) {
    return val;
}
int DX1Motor::convertToTemp(int raw) {
    return raw;
}

int DX1Motor::convertFromVoltage(float val) {
    return (int) val*10;
}
float DX1Motor::convertToVoltage(int raw) {
    return 0.1f * raw;
}

int DX1Motor::convertFromDegrees(float val) {
    return (int)(val * 1023.0f / 300);
}
float DX1Motor::convertToDegrees(int raw) {
    return 300.0f / 1023 * raw;
}

int DX1Motor::convertFromPercentage(float val) {
    int base = 1024;
    if (val < 0) {
        base = 0;
        val = fabs(val);
    }
    return base + (int)(val * 1023);
}
float DX1Motor::convertToPercentage(int raw) {
    if (raw < 1024) {
        return (-1.0f/1023) * raw;
    }
    else {
        return (1.0f/1023) * (raw - 1024);
    }
}

// Basic checksum
unsigned char DX1Motor::computeChecksum(unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned char accum = 0;

    for (int j = 0; j < data_blk_size; j++) {
        accum += data_blk_ptr[j];
    }

    return ~accum;
}

void DX1Motor::startPacket(unsigned char instruction, bool broadcast) {
    // Begin tracking packet size and input data
    // Length is size of parameter block + 2 (instr and checksum)
    _packet_length = 2;
    // Create the packet buffer
    if (_packet_data != NULL) {
        delete _packet_data;
    }
    _packet_data = new unsigned char[DX1MOTOR_PACKET_BUFFER_SIZE];

    // Start filling the packet
    _packet_data[0] = 0xFF;
    _packet_data[1] = 0xFF;
    _packet_data[2] = broadcast ? DX1MOTOR_BROADCAST_ID : _id;
    // [3] will contain the packet length
    _packet_data[4] = instruction;

    // Wipe any leftover response info
    _responseParams = 0;
}

void DX1Motor::bufferParams(unsigned char *data_block, unsigned short block_length) {
    for (int i = 0; i < block_length; i++) {
        _packet_data[3 + _packet_length] = data_block[i];
        _packet_length++;
    }
}

void DX1Motor::sendPacket() {
    digitalWrite(CommPin, 1);

    // Store the completed packet length
    _packet_data[3] = _packet_length;

    // Checksum error bits (misses out header)
    unsigned char checksum = computeChecksum(_packet_data + 2, _packet_length + 1);
    _packet_data[_packet_length + 3] = checksum;

    // Write out the buffered packet
    for (int i = 0; i < _packet_length + 4; i++) {
        unsigned char b = _packet_data[i];
        TemplateSerial.write(b);
    }
    //TemplateSerial.write((char*) _packet_data, _packet_length + 7);
    //TemplateSerial.flush();
    //delayMicroseconds(DX1MOTOR_TX_DELAY_TIME);
    // for (int i = 0; i < 5; i++) {
    //     TemplateSerial.write(0);
    // }
    // Ensure output completed before we switch comm modes
    TemplateSerial.flush();
    // Clear junk bytes
    //while (TemplateSerial.available())
    //    TemplateSerial.read();
    digitalWrite(CommPin, 0);

    // Clear the packet buffer
    delete _packet_data;
    _packet_data = NULL;
}

int DX1Motor::doReceive(int timeout) {
    int err = 0; // return val
    int responseError = 0;
    _responseParams = 0;

    unsigned char pattern[2];
    pattern[0] = 0xFF;
    pattern[1] = 0xFF;
    int match = 0;

    unsigned char block[256];
    bool foundHeader = false;
    unsigned short len = 256;

    unsigned char debug_buffer[256];
    int debug_ind = 0;

    // Pattern-matching header with timeout
    // Breaks immediately when entire length has been read
    while (waitTimeout(timeout)) {
        unsigned char b = TemplateSerial.read();
        debug_buffer[debug_ind++] = b;

        // Match the header
        if (match < 2) {
            if (b == pattern[match]) {
                match++;
            }
            else if (b != 0xFF) {
                match = 0;
            }
        }
        // Store data
        // block[0] = id
        // block[1] = length
        else if (match - 2 < 256) {
            foundHeader = true;
            block[match - 2] = b;
            if (match == 3) {
                len = b;
            }
            else if (match == len + 3) {
                break;
            }
            match++;
        }
        else {
            break;
        }
    }

    if (!foundHeader) {
        // We timed out
        return DX1MOTOR_ERR_TIMEOUT;
    }

    // Corruptions (including of ID) should be caught by checksum
    int ind = 0;
    unsigned char responseID = block[ind++];
    if (responseID != _id) {
        /*Serial.print("RESPONSE ERROR: Unexpected ID ");
        Serial.println(responseID, DEC);
        Serial.print("Received ");
        Serial.write(debug_buffer, debug_ind);
        err |= DX1MOTOR_ERR_PROTOCOL;*/
    }

    ind++;
    // Get remaining response length & wait for buffer to fill, if necessary
    _responseParams = len - 2;

    // Error flags
    responseError = block[ind++];

    // Parameters are stored up to the capacity limit
    for (int i = 0; i < _responseParams; i++) {
        unsigned char byte = block[ind++];
        if (i < 16) {
            _responseData[i] = byte;
        }
    }

    // Verify the checksum (misses header)
    unsigned char received_chk = block[ind];
    if (received_chk != computeChecksum(block, ind)) {
        //Serial.println("Corruption");
        return DX1MOTOR_ERR_CORRUPTION;
    }


    // Check the status-error flags
    if (responseError != 0) {
        if (responseError & (1 << 0)) {
            err |= DX1MOTOR_ERR_INPUT_VOLTAGE;
        }
        if (responseError & (1 << 1)) {
            err |= DX1MOTOR_ERR_ANGLE_LIMIT;
        }
        if (responseError & (1 << 2)) {
            err |= DX1MOTOR_ERR_OVERHEAT;
        }
        if (responseError & (1 << 3)) {
            err |= DX1MOTOR_ERR_RANGE_LIMIT;
        }
        if (responseError & (1 << 4)) {
            err |= DX1MOTOR_ERR_BAD_CHECKSUM;
        }
        if (responseError & (1 << 5)) {
            err |= DX1MOTOR_ERR_OVERLOAD;
        }
        if (responseError & (1 << 6)) {
            err |= DX1MOTOR_ERR_INSTRUCTION;
        }

    }

    return err;
}

int DX1Motor::read(unsigned char adr, unsigned char len, unsigned char *data) {
    while (true) {
        unsigned char b[2];
        startPacket(DX1MOTOR_READ_DATA);
        b[0] = adr;
        b[1] = len;
        bufferParams(b, 2);
        sendPacket();

        // Get the response
        int err = 0;
        err = doReceive(DX1MOTOR_RX_TIMEOUT);

        if (err == DX1MOTOR_ERR_CORRUPTION) {
            // Packet got corrupted, try again
            continue;
        }

        if (!(err & DX1MOTOR_ERR_TIMEOUT) && _responseParams == len) {
            for (int i = 0; i < len; i++) {
                data[i] = _responseData[i];
            }
        }

        return err;
    }
}

int DX1Motor::write(unsigned char adr, unsigned char len, unsigned char *data) {
    while (true) {
        unsigned char b[1];
        startPacket(DX1MOTOR_WRITE_DATA);
        b[0] = adr;
        bufferParams(b, 1);
        bufferParams(data, len);
        sendPacket();

        // Low return level
        return 0;
        /*int err = doReceive(DX1MOTOR_RX_TIMEOUT);
        if (err == DX1MOTOR_ERR_CORRUPTION) {
            // Packet got corrupted, try again
            continue;
        }
        return err;*/
    }
}//// END INCLUDE

int DX1Motor::getModelNumber(int &err) {
    unsigned char b[2];
    err = read(0, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::getFirmwareVersion(int &err) {
    unsigned char b[1];
    err = read(2, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::getID(int &err) {
    unsigned char b[1];
    err = read(3, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setID(int id) {
    unsigned char b[1];
    int raw = id;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 253) {
        raw = 253;
    }
    b[0] = raw;
    int err = write(3, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        // Update internal ID
        _id = id;
    }
    return err;
}

int DX1Motor::getBaudRate(int &err) {
    unsigned char b[1];
    err = read(4, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setBaudRate(int baudrate) {
    unsigned char b[1];
    int raw = baudrate;
    b[0] = raw;
    int err = write(4, 1, b);
    return err;
}

int DX1Motor::getReturnDelay(int &err) {
    unsigned char b[1];
    err = read(5, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setReturnDelay(int returndelay) {
    unsigned char b[1];
    int raw = returndelay;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 254) {
        raw = 254;
    }
    b[0] = raw;
    int err = write(5, 1, b);
    return err;
}

float DX1Motor::getCWLimit(int &err) {
    unsigned char b[2];
    err = read(6, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setCWLimit(float cwlimit) {
    unsigned char b[2];
    int raw = convertFromDegrees(cwlimit);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(6, 2, b);
    return err;
}

float DX1Motor::getCCWLimit(int &err) {
    unsigned char b[2];
    err = read(8, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setCCWLimit(float ccwlimit) {
    unsigned char b[2];
    int raw = convertFromDegrees(ccwlimit);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(8, 2, b);
    return err;
}

int DX1Motor::getTempLimit(int &err) {
    unsigned char b[1];
    err = read(11, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToTemp(raw);
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setTempLimit(int templimit) {
    unsigned char b[1];
    int raw = convertFromTemp(templimit);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 150) {
        raw = 150;
    }
    b[0] = raw;
    int err = write(11, 1, b);
    return err;
}

float DX1Motor::getLowVoltageLimit(int &err) {
    unsigned char b[1];
    err = read(12, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setLowVoltageLimit(float lowvoltagelimit) {
    unsigned char b[1];
    int raw = convertFromVoltage(lowvoltagelimit);
    if (raw < 50) {
        raw = 50;
    }
    if (raw > 250) {
        raw = 250;
    }
    b[0] = raw;
    int err = write(12, 1, b);
    return err;
}

float DX1Motor::getHighVoltageLimit(int &err) {
    unsigned char b[1];
    err = read(13, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setHighVoltageLimit(float highvoltagelimit) {
    unsigned char b[1];
    int raw = convertFromVoltage(highvoltagelimit);
    if (raw < 50) {
        raw = 50;
    }
    if (raw > 250) {
        raw = 250;
    }
    b[0] = raw;
    int err = write(13, 1, b);
    return err;
}

float DX1Motor::getMaxTorque(int &err) {
    unsigned char b[2];
    err = read(14, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setMaxTorque(float maxtorque) {
    unsigned char b[2];
    int raw = convertFromPercentage(maxtorque);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(14, 2, b);
    return err;
}

int DX1Motor::getReturnLevel(int &err) {
    unsigned char b[1];
    err = read(16, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setReturnLevel(int returnlevel) {
    unsigned char b[1];
    int raw = returnlevel;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 2) {
        raw = 2;
    }
    b[0] = raw;
    int err = write(16, 1, b);
    return err;
}

int DX1Motor::getAlarmFlags(int &err) {
    unsigned char b[1];
    err = read(17, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setAlarmFlags(int alarmflags) {
    unsigned char b[1];
    int raw = alarmflags;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 127) {
        raw = 127;
    }
    b[0] = raw;
    int err = write(17, 1, b);
    return err;
}

int DX1Motor::getShutdownFlags(int &err) {
    unsigned char b[1];
    err = read(18, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setShutdownFlags(int shutdownflags) {
    unsigned char b[1];
    int raw = shutdownflags;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 127) {
        raw = 127;
    }
    b[0] = raw;
    int err = write(18, 1, b);
    return err;
}

bool DX1Motor::getTorqueEnable(int &err) {
    unsigned char b[1];
    err = read(24, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

int DX1Motor::setTorqueEnable(bool torqueenable) {
    unsigned char b[1];
    int raw = torqueenable;
    b[0] = raw;
    int err = write(24, 1, b);
    return err;
}

int DX1Motor::getLED(int &err) {
    unsigned char b[1];
    err = read(25, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setLED(int led) {
    unsigned char b[1];
    int raw = led;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1) {
        raw = 1;
    }
    b[0] = raw;
    int err = write(25, 1, b);
    return err;
}

int DX1Motor::getCWMargin(int &err) {
    unsigned char b[1];
    err = read(26, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setCWMargin(int cwmargin) {
    unsigned char b[1];
    int raw = cwmargin;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 254) {
        raw = 254;
    }
    b[0] = raw;
    int err = write(26, 1, b);
    return err;
}

int DX1Motor::getCCWMargin(int &err) {
    unsigned char b[1];
    err = read(27, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setCCWMargin(int ccwmargin) {
    unsigned char b[1];
    int raw = ccwmargin;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 254) {
        raw = 254;
    }
    b[0] = raw;
    int err = write(27, 1, b);
    return err;
}

int DX1Motor::getCWSlope(int &err) {
    unsigned char b[1];
    err = read(28, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setCWSlope(int cwslope) {
    unsigned char b[1];
    int raw = cwslope;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 254) {
        raw = 254;
    }
    b[0] = raw;
    int err = write(28, 1, b);
    return err;
}

int DX1Motor::getCCWSlope(int &err) {
    unsigned char b[1];
    err = read(29, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setCCWSlope(int ccwslope) {
    unsigned char b[1];
    int raw = ccwslope;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 254) {
        raw = 254;
    }
    b[0] = raw;
    int err = write(29, 1, b);
    return err;
}

float DX1Motor::getGoalPosition(int &err) {
    unsigned char b[2];
    err = read(30, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setGoalPosition(float goalposition) {
    unsigned char b[2];
    int raw = convertFromDegrees(goalposition);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(30, 2, b);
    return err;
}

float DX1Motor::getGoalSpeed(int &err) {
    unsigned char b[2];
    err = read(32, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setGoalSpeed(float goalspeed) {
    unsigned char b[2];
    int raw = convertFromPercentage(goalspeed);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(32, 2, b);
    return err;
}

float DX1Motor::getTorqueLimit(int &err) {
    unsigned char b[2];
    err = read(34, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setTorqueLimit(float torquelimit) {
    unsigned char b[2];
    int raw = convertFromPercentage(torquelimit);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(34, 2, b);
    return err;
}

float DX1Motor::getPosition(int &err) {
    unsigned char b[2];
    err = read(36, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX1Motor::getSpeed(int &err) {
    unsigned char b[2];
    err = read(38, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX1Motor::getLoad(int &err) {
    unsigned char b[2];
    err = read(40, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX1Motor::getVoltage(int &err) {
    unsigned char b[1];
    err = read(42, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::getTemperature(int &err) {
    unsigned char b[1];
    err = read(43, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToTemp(raw);
    }
    else {
        return (int) NULL;
    }
}

bool DX1Motor::getInstructionStored(int &err) {
    unsigned char b[1];
    err = read(44, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

bool DX1Motor::getMoving(int &err) {
    unsigned char b[1];
    err = read(46, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

int DX1Motor::getLock(int &err) {
    unsigned char b[1];
    err = read(47, 1, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX1Motor::setLock(int lock) {
    unsigned char b[1];
    int raw = lock;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1) {
        raw = 1;
    }
    b[0] = raw;
    int err = write(47, 1, b);
    return err;
}

float DX1Motor::getPunch(int &err) {
    unsigned char b[2];
    err = read(48, 2, b);
    if (err == DX1MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX1Motor::setPunch(float punch) {
    unsigned char b[2];
    int raw = convertFromPercentage(punch);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    b[0] = raw;
    b[1] = raw >> 8;
    int err = write(48, 2, b);
    return err;
}


#undef TemplateSerial