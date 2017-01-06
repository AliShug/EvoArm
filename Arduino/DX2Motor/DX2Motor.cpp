/* Do not modify - auto-generated code file
 * Produced by jinjagen.py

 * DESCRIPTION
 * Interface between an arduino microcontroller and Dynamixel servo motors.
 */

#include "DX2Motor.h"

#define TemplateSerial Serial2

int DX2Motor::CommPin;
long DX2Motor::Baud;

//// INCLUDED FROM dx2_codefile.cpp
bool waitTimeout(int timeout = 16) {
    unsigned long startTime = micros();

    do {
        if (TemplateSerial.peek() >= 0) {
            return true;
        }
    } while (micros() - startTime < timeout);

    return false;
}

// Must be called before any instance use
void DX2Motor::Init(int baud_mode, int pin) {
    CommPin = pin;

    switch (baud_mode) {
    case DX2MOTOR_BAUD_9600:
        Baud = 9600;
        break;
    case DX2MOTOR_BAUD_57600:
        Baud = 57600;
        break;
    case DX2MOTOR_BAUD_115200:
        Baud = 115200;
        break;
    case DX2MOTOR_BAUD_1MBS:
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

void DX2Motor::retarget(unsigned char id) {
    _id = id;
}

DX2Motor::DX2Motor(unsigned char id) {
    _id = id;
    _packet_data = NULL;
}

DX2Motor::DX2Motor() {
    _id = 0;
    _packet_data = NULL;
}

DX2Motor::~DX2Motor() {
    if (_packet_data != NULL) {
        delete _packet_data;
        _packet_data = NULL;
    }
}

// Unit Conversions
int DX2Motor::convertFromTemp(int val) {
    return val;
}
int DX2Motor::convertToTemp(int raw) {
    return raw;
}

int DX2Motor::convertFromVoltage(float val) {
    return (int) val*10;
}
float DX2Motor::convertToVoltage(int raw) {
    return 0.1f * raw;
}

int DX2Motor::convertFromDegrees(float val) {
    return (int)(val * 1023.0f / 300);
}
float DX2Motor::convertToDegrees(int raw) {
    return 300.0f / 1023 * raw;
}

int DX2Motor::convertFromPercentage(float val) {
    int base = 1024;
    if (val < 0) {
        base = 0;
        val = fabs(val);
    }
    return base + (int)(val * 1023);
}
float DX2Motor::convertToPercentage(int raw) {
    if (raw < 1024) {
        return (-1.0f/1023) * raw;
    }
    else {
        return (1.0f/1023) * (raw - 1024);
    }
}

// 16-bit cyclic redundancy check (based on Robotis-published code)
unsigned short DX2Motor::computeCRC(unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short crc_accum = 0;
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void DX2Motor::startPacket(unsigned char instruction, bool broadcast) {
    // Begin tracking packet size and input data
    // Length is size of parameter block + 3 (instr and 16-bit CRC)
    _packet_length = 3;
    // Create the packet buffer
    if (_packet_data != NULL) {
        delete _packet_data;
    }
    _packet_data = new unsigned char[DX2MOTOR_PACKET_BUFFER_SIZE];

    // Start filling the packet
    _packet_data[0] = 0xFF;
    _packet_data[1] = 0xFF;
    _packet_data[2] = 0xFD;
    _packet_data[3] = 0x00; // 'reserved'
    _packet_data[4] = broadcast ? DX2MOTOR_BROADCAST_ID : _id;
    // [5-6] will contain the packet length
    _packet_data[7] = instruction;

    // Wipe any leftover response info
    _responseParams = 0;
}

void DX2Motor::bufferParams(unsigned char *data_block, unsigned short block_length) {
    for (int i = 0; i < block_length; i++) {
        _packet_data[5 + _packet_length] = data_block[i];
        _packet_length++;
    }
}

void DX2Motor::sendPacket() {
    digitalWrite(CommPin, 1);

    // Store the completed packet length
    _packet_data[5] = _packet_length;
    _packet_data[6] = _packet_length >> 8;

    // CRC error bits
    unsigned short crc = computeCRC(_packet_data, _packet_length + 5);
    _packet_data[_packet_length + 5] = crc;
    _packet_data[_packet_length + 6] = crc >> 8;

    // Write out the buffered packet
    for (int i = 0; i < _packet_length + 7; i++) {
        unsigned char b = _packet_data[i];
        TemplateSerial.write(b);
    }
    //TemplateSerial.write((char*) _packet_data, _packet_length + 7);
    //TemplateSerial.flush();

    // Ensure output completed before we switch comm modes
    //TemplateSerial.flush(); // hangs sometimes...
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    digitalWrite(CommPin, 0);
    // Clear 'reflected' bytes
    while (TemplateSerial.available())
        TemplateSerial.read();

    // Clear the packet buffer
    delete _packet_data;
    _packet_data = NULL;
}

int DX2Motor::doReceive() {
    int err = 0; // return val
    int responseError = 0;
    _responseParams = 0;

    unsigned char pattern[3];
    pattern[0] = 0xFF;
    pattern[1] = 0xFF;
    pattern[2] = 0xFD;
    int match = 0;

    unsigned char block[1024];
    bool foundHeader = false;
    unsigned short len = 1024;

    // Pattern-matching header with timeout
    while (waitTimeout(DX2MOTOR_RX_TIMEOUT)) {
        unsigned char b = TemplateSerial.read();

        // Match the header
        if (match < 3) {
            if (b == pattern[match]) {
                match++;
            }
            else if (b != 0xFF) {
                match = 0;
            }
        }
        // Store data
        else if (match - 3 < 1024) {
            foundHeader = true;
            block[match - 3] = b;
            if (match == 6) {
                len = block[2];
                len |= block[3] << 8;
            }
            else if (match == len + 6) {
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
        return DX2MOTOR_ERR_TIMEOUT;
    }

    // Skip reserved byte
    int ind = 1;

    // Verify ID
    unsigned char responseID = block[ind++];
    if (responseID != _id) {
        err |= DX2MOTOR_ERR_PROTOCOL;
    }
    else {
        // already got length
        ind += 2;

        // Instruction byte
        unsigned char instr = block[ind++];

        if (len <= 64) {
            _responseParams = len - 4;

            // Error flags
            int responseError = block[ind++];

            // Parameters are stored up to the capacity limit
            for (int i = 0; i < _responseParams; i++) {
                unsigned char byte = block[ind++];
                if (i < 16) {
                    _responseData[i] = byte;
                }
            }
        }
    }

    // Check the status-error flags
    if (responseError != 0) {
        if (responseError & 1) {
            err |= DX2MOTOR_ERR_EXECUTION;
        }
        if (responseError & 2) {
            err |= DX2MOTOR_ERR_INSTRUCTION;
        }
        if (responseError & 4) {
            err |= DX2MOTOR_ERR_CRC;
        }
        if (responseError & 8) {
            err |= DX2MOTOR_ERR_RANGE;
        }
        if (responseError & 16) {
            err |= DX2MOTOR_ERR_OVERLENGTH;
        }
        if (responseError & 32) {
            err |= DX2MOTOR_ERR_UNDERLENGTH;
        }
        if (responseError & 64) {
            err |= DX2MOTOR_ERR_ACCESS;
        }
        if (responseError & 128) {
            err |= DX2MOTOR_ERR_HARDWARE;
        }

    }

    return err;
}

int DX2Motor::read(unsigned short adr, unsigned short len, unsigned char *data) {
    unsigned char b[2];
    startPacket(DX2MOTOR_READ_DATA);
    b[0] = adr;
    b[1] = adr >> 8;
    bufferParams(b, 2);
    b[0] = len;
    b[1] = len >> 8;
    bufferParams(b, 2);
    sendPacket();

    // Get the response
    int err = 0;
    err = doReceive();

    if (!(err & DX2MOTOR_ERR_TIMEOUT) && _responseParams == len) {
        for (int i = 0; i < len; i++) {
            data[i] = _responseData[i];
        }
    }

    return err;
}

int DX2Motor::write(unsigned short adr, unsigned short len, unsigned char *data) {
    unsigned char b[2];
    startPacket(DX2MOTOR_WRITE_DATA);
    b[0] = adr;
    b[1] = adr >> 8;
    bufferParams(b, 2);
    bufferParams(data, len);
    sendPacket();

    return doReceive();
}//// END INCLUDE

int DX2Motor::getModelNumber(int &err) {
    unsigned char b[2];
    err = read(0, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::getFirmwareVersion(int &err) {
    unsigned char b[1];
    err = read(2, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::getID(int &err) {
    unsigned char b[1];
    err = read(3, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setID(int id) {
    unsigned char b[1];
    int raw = id;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 252) {
        raw = 252;
    }
    b[0] = raw;
    int err = write(3, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        // Update internal ID
        _id = id;
    }
    return err;
}

int DX2Motor::getBaudRate(int &err) {
    unsigned char b[1];
    err = read(4, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setBaudRate(int baudrate) {
    unsigned char b[1];
    int raw = baudrate;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 3) {
        raw = 3;
    }
    b[0] = raw;
    int err = write(4, 1, b);
    return err;
}

int DX2Motor::getReturnDelay(int &err) {
    unsigned char b[1];
    err = read(5, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setReturnDelay(int returndelay) {
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

float DX2Motor::getCWLimit(int &err) {
    unsigned char b[2];
    err = read(6, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setCWLimit(float cwlimit) {
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

float DX2Motor::getCCWLimit(int &err) {
    unsigned char b[2];
    err = read(8, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setCCWLimit(float ccwlimit) {
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

int DX2Motor::getControlMode(int &err) {
    unsigned char b[1];
    err = read(11, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setControlMode(int controlmode) {
    unsigned char b[1];
    int raw = controlmode;
    if (raw < 1) {
        raw = 1;
    }
    if (raw > 2) {
        raw = 2;
    }
    b[0] = raw;
    int err = write(11, 1, b);
    return err;
}

int DX2Motor::getTempLimit(int &err) {
    unsigned char b[1];
    err = read(12, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToTemp(raw);
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setTempLimit(int templimit) {
    unsigned char b[1];
    int raw = convertFromTemp(templimit);
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 150) {
        raw = 150;
    }
    b[0] = raw;
    int err = write(12, 1, b);
    return err;
}

float DX2Motor::getLowVoltageLimit(int &err) {
    unsigned char b[1];
    err = read(13, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setLowVoltageLimit(float lowvoltagelimit) {
    unsigned char b[1];
    int raw = convertFromVoltage(lowvoltagelimit);
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

float DX2Motor::getHighVoltageLimit(int &err) {
    unsigned char b[1];
    err = read(14, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setHighVoltageLimit(float highvoltagelimit) {
    unsigned char b[1];
    int raw = convertFromVoltage(highvoltagelimit);
    if (raw < 50) {
        raw = 50;
    }
    if (raw > 250) {
        raw = 250;
    }
    b[0] = raw;
    int err = write(14, 1, b);
    return err;
}

float DX2Motor::getMaxTorque(int &err) {
    unsigned char b[2];
    err = read(15, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setMaxTorque(float maxtorque) {
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
    int err = write(15, 2, b);
    return err;
}

int DX2Motor::getReturnLevel(int &err) {
    unsigned char b[1];
    err = read(17, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setReturnLevel(int returnlevel) {
    unsigned char b[1];
    int raw = returnlevel;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 2) {
        raw = 2;
    }
    b[0] = raw;
    int err = write(17, 1, b);
    return err;
}

int DX2Motor::getShutdownFlags(int &err) {
    unsigned char b[1];
    err = read(18, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setShutdownFlags(int shutdownflags) {
    unsigned char b[1];
    int raw = shutdownflags;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 7) {
        raw = 7;
    }
    b[0] = raw;
    int err = write(18, 1, b);
    return err;
}

bool DX2Motor::getTorqueEnable(int &err) {
    unsigned char b[1];
    err = read(24, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

int DX2Motor::setTorqueEnable(bool torqueenable) {
    unsigned char b[1];
    int raw = torqueenable;
    b[0] = raw;
    int err = write(24, 1, b);
    return err;
}

int DX2Motor::getLED(int &err) {
    unsigned char b[1];
    err = read(25, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setLED(int led) {
    unsigned char b[1];
    int raw = led;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 7) {
        raw = 7;
    }
    b[0] = raw;
    int err = write(25, 1, b);
    return err;
}

int DX2Motor::getDGain(int &err) {
    unsigned char b[1];
    err = read(27, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setDGain(int dgain) {
    unsigned char b[1];
    int raw = dgain;
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

int DX2Motor::getIGain(int &err) {
    unsigned char b[1];
    err = read(28, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setIGain(int igain) {
    unsigned char b[1];
    int raw = igain;
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

int DX2Motor::getPGain(int &err) {
    unsigned char b[1];
    err = read(29, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

int DX2Motor::setPGain(int pgain) {
    unsigned char b[1];
    int raw = pgain;
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

float DX2Motor::getGoalPosition(int &err) {
    unsigned char b[2];
    err = read(30, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setGoalPosition(float goalposition) {
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

float DX2Motor::getGoalSpeed(int &err) {
    unsigned char b[2];
    err = read(32, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setGoalSpeed(float goalspeed) {
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

float DX2Motor::getTorqueLimit(int &err) {
    unsigned char b[2];
    err = read(35, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setTorqueLimit(float torquelimit) {
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
    int err = write(35, 2, b);
    return err;
}

float DX2Motor::getPosition(int &err) {
    unsigned char b[2];
    err = read(37, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToDegrees(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX2Motor::getSpeed(int &err) {
    unsigned char b[2];
    err = read(39, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX2Motor::getLoad(int &err) {
    unsigned char b[2];
    err = read(41, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

float DX2Motor::getVoltage(int &err) {
    unsigned char b[1];
    err = read(45, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToVoltage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::getTemperature(int &err) {
    unsigned char b[1];
    err = read(46, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return convertToTemp(raw);
    }
    else {
        return (int) NULL;
    }
}

bool DX2Motor::getInstructionStored(int &err) {
    unsigned char b[1];
    err = read(47, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

bool DX2Motor::getMoving(int &err) {
    unsigned char b[1];
    err = read(49, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (bool) NULL;
    }
}

int DX2Motor::getHardwareError(int &err) {
    unsigned char b[1];
    err = read(50, 1, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        return raw;
    }
    else {
        return (int) NULL;
    }
}

float DX2Motor::getPunch(int &err) {
    unsigned char b[2];
    err = read(51, 2, b);
    if (err == DX2MOTOR_ERR_OK) {
        int raw = b[0];
        raw |= b[1] << 8;
        return convertToPercentage(raw);
    }
    else {
        return (float) NULL;
    }
}

int DX2Motor::setPunch(float punch) {
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
    int err = write(51, 2, b);
    return err;
}


#undef TemplateSerial