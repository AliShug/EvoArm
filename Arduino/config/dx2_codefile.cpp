bool waitTimeout(int timeout = 16) {
    unsigned long startTime = micros();

    do {
        if (TemplateSerial.peek() >= 0) {
            return true;
        }
    } while (micros() - startTime < timeout);

    return false;
}

{% set cname = data.classname %}
// Must be called before any instance use
void {{cname}}::Init(int baud_mode, int pin) {
    CommPin = pin;

    switch (baud_mode) {
    case {{cname|upper}}_BAUD_9600:
        Baud = 9600;
        break;
    case {{cname|upper}}_BAUD_57600:
        Baud = 57600;
        break;
    case {{cname|upper}}_BAUD_115200:
        Baud = 115200;
        break;
    case {{cname|upper}}_BAUD_1MBS:
    default:
        Baud = 1000000;
        break;
    }

    {% if data.debug %}
    Serial.print("Baud: ");
    Serial.println(Baud);
    {% endif %}

    pinMode(CommPin, OUTPUT);
    digitalWrite(CommPin, 1);
    TemplateSerial.begin(Baud);
    while (!TemplateSerial) ;
    TemplateSerial.setTimeout(1);
}

void {{cname}}::retarget(unsigned char id) {
    _id = id;
}

{{cname}}::{{cname}}(unsigned char id) {
    _id = id;
    _packet_data = NULL;
}

{{cname}}::{{cname}}() {
    _id = 0;
    _packet_data = NULL;
}

{{cname}}::~{{cname}}() {
    if (_packet_data != NULL) {
        delete _packet_data;
        _packet_data = NULL;
    }
}

// Unit Conversions
int {{cname}}::convertFromTemp(int val) {
    return val;
}
int {{cname}}::convertToTemp(int raw) {
    return raw;
}

int {{cname}}::convertFromVoltage(float val) {
    return (int) val*10;
}
float {{cname}}::convertToVoltage(int raw) {
    return 0.1f * raw;
}

int {{cname}}::convertFromDegrees(float val) {
    return (int)(val * 1023.0f / 300);
}
float {{cname}}::convertToDegrees(int raw) {
    return 300.0f / 1023 * raw;
}

int {{cname}}::convertFromPercentage(float val) {
    int base = 1024;
    if (val < 0) {
        base = 0;
        val = fabs(val);
    }
    return base + (int)(val * 1023);
}
float {{cname}}::convertToPercentage(int raw) {
    if (raw < 1024) {
        return (-1.0f/1023) * raw;
    }
    else {
        return (1.0f/1023) * (raw - 1024);
    }
}

// 16-bit cyclic redundancy check (based on Robotis-published code)
unsigned short {{cname}}::computeCRC(unsigned char *data_blk_ptr, unsigned short data_blk_size) {
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

void {{cname}}::startPacket(unsigned char instruction, bool broadcast) {
    // Begin tracking packet size and input data
    // Length is size of parameter block + 3 (instr and 16-bit CRC)
    _packet_length = 3;
    // Create the packet buffer
    if (_packet_data != NULL) {
        delete _packet_data;
    }
    _packet_data = new unsigned char[{{cname|upper}}_PACKET_BUFFER_SIZE];

    // Start filling the packet
    _packet_data[0] = 0xFF;
    _packet_data[1] = 0xFF;
    _packet_data[2] = 0xFD;
    _packet_data[3] = 0x00; // 'reserved'
    _packet_data[4] = broadcast ? {{cname|upper}}_BROADCAST_ID : _id;
    // [5-6] will contain the packet length
    _packet_data[7] = instruction;

    // Wipe any leftover response info
    _responseParams = 0;
}

void {{cname}}::bufferParams(unsigned char *data_block, unsigned short block_length) {
    for (int i = 0; i < block_length; i++) {
        _packet_data[5 + _packet_length] = data_block[i];
        _packet_length++;
    }
}

void {{cname}}::sendPacket() {
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

int {{cname}}::doReceive() {
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
    while (waitTimeout({{cname|upper}}_RX_TIMEOUT)) {
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
        {% if data.debug %}
        Serial.println("RESPONSE TIMEOUT");
        {% endif %}
        return {{cname|upper}}_ERR_TIMEOUT;
    }

    // Skip reserved byte
    int ind = 1;

    // Verify ID
    unsigned char responseID = block[ind++];
    if (responseID != _id) {
        {% if data.debug %}
        Serial.print("RESPONSE ERROR: Unexpected ID ");
        {% endif %}
        {% if data.debug %}
        Serial.println(responseID, DEC);
        {% endif %}
        err |= {{cname|upper}}_ERR_PROTOCOL;
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
        {% if data.debug %}
        Serial.print("STATUS ERROR: ");
        {% endif %}
        if (responseError & 1) {
            {% if data.debug %}
            Serial.print("EXECUTION_FAILURE ");
            {% endif %}
            err |= {{cname|upper}}_ERR_EXECUTION;
        }
        if (responseError & 2) {
            {% if data.debug %}
            Serial.print("BAD_INSTRUCTION ");
            {% endif %}
            err |= {{cname|upper}}_ERR_INSTRUCTION;
        }
        if (responseError & 4) {
            {% if data.debug %}
            Serial.print("BAD_CRC ");
            {% endif %}
            err |= {{cname|upper}}_ERR_CRC;
        }
        if (responseError & 8) {
            {% if data.debug %}
            Serial.print("RANGE_LIM ");
            {% endif %}
            err |= {{cname|upper}}_ERR_RANGE;
        }
        if (responseError & 16) {
            {% if data.debug %}
            Serial.print("OVERLENGTH ");
            {% endif %}
            err |= {{cname|upper}}_ERR_OVERLENGTH;
        }
        if (responseError & 32) {
            {% if data.debug %}
            Serial.print("UNDERLENGTH ");
            {% endif %}
            err |= {{cname|upper}}_ERR_UNDERLENGTH;
        }
        if (responseError & 64) {
            {% if data.debug %}
            Serial.print("ILLEGAL_ACCESS ");
            {% endif %}
            err |= {{cname|upper}}_ERR_ACCESS;
        }
        if (responseError & 128) {
            {% if data.debug %}
            Serial.print("HARDWARE_ALERT ");
            {% endif %}
            err |= {{cname|upper}}_ERR_HARDWARE;
        }

        {% if data.debug %}
        Serial.println("|");
        {% endif %}
    }

    return err;
}

int {{cname}}::read(unsigned short adr, unsigned short len, unsigned char *data) {
    unsigned char b[2];
    startPacket({{cname|upper}}_READ_DATA);
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

    if (!(err & {{cname|upper}}_ERR_TIMEOUT) && _responseParams == len) {
        for (int i = 0; i < len; i++) {
            data[i] = _responseData[i];
        }
    }

    return err;
}

int {{cname}}::write(unsigned short adr, unsigned short len, unsigned char *data) {
    unsigned char b[2];
    startPacket({{cname|upper}}_WRITE_DATA);
    b[0] = adr;
    b[1] = adr >> 8;
    bufferParams(b, 2);
    bufferParams(data, len);
    sendPacket();

    return doReceive();
}
