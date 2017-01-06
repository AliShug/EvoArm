{% set cname = data.classname %}

bool {{cname}}::waitTimeout(int timeout) {
    unsigned long startTime = micros();

    do {
        if (TemplateSerial.peek() >= 0) {
            return true;
        }
    } while (micros() - startTime < timeout);

    return false;
}

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

// Basic checksum
unsigned char {{cname}}::computeChecksum(unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned char accum = 0;

    for (int j = 0; j < data_blk_size; j++) {
        accum += data_blk_ptr[j];
    }

    return ~accum;
}

void {{cname}}::startPacket(unsigned char instruction, bool broadcast) {
    // Begin tracking packet size and input data
    // Length is size of parameter block + 2 (instr and checksum)
    _packet_length = 2;
    // Create the packet buffer
    if (_packet_data != NULL) {
        delete _packet_data;
    }
    _packet_data = new unsigned char[{{cname|upper}}_PACKET_BUFFER_SIZE];

    // Start filling the packet
    _packet_data[0] = 0xFF;
    _packet_data[1] = 0xFF;
    _packet_data[2] = broadcast ? {{cname|upper}}_BROADCAST_ID : _id;
    // [3] will contain the packet length
    _packet_data[4] = instruction;

    // Wipe any leftover response info
    _responseParams = 0;
}

void {{cname}}::bufferParams(unsigned char *data_block, unsigned short block_length) {
    for (int i = 0; i < block_length; i++) {
        _packet_data[3 + _packet_length] = data_block[i];
        _packet_length++;
    }
}

void {{cname}}::sendPacket() {
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
    //delayMicroseconds({{cname|upper}}_TX_DELAY_TIME);
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

int {{cname}}::doReceive(int timeout) {
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
        {% if data.debug %}
        Serial.println("RESPONSE TIMEOUT");
        {% endif %}
        return {{cname|upper}}_ERR_TIMEOUT;
    }

    // Corruptions (including of ID) should be caught by checksum
    int ind = 0;
    unsigned char responseID = block[ind++];
    if (responseID != _id) {
        /*Serial.print("RESPONSE ERROR: Unexpected ID ");
        Serial.println(responseID, DEC);
        Serial.print("Received ");
        Serial.write(debug_buffer, debug_ind);
        err |= {{cname|upper}}_ERR_PROTOCOL;*/
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
        return {{cname|upper}}_ERR_CORRUPTION;
    }


    // Check the status-error flags
    if (responseError != 0) {
        {% if data.debug %}
        Serial.print("STATUS ERROR: ");
        {% endif %}
        if (responseError & (1 << 0)) {
            {% if data.debug %}
            Serial.print("INPUT_VOLTAGE ");
            {% endif %}
            err |= {{cname|upper}}_ERR_INPUT_VOLTAGE;
        }
        if (responseError & (1 << 1)) {
            {% if data.debug %}
            Serial.print("ANGLE_LIMIT ");
            {% endif %}
            err |= {{cname|upper}}_ERR_ANGLE_LIMIT;
        }
        if (responseError & (1 << 2)) {
            {% if data.debug %}
            Serial.print("OVERHEAT ");
            {% endif %}
            err |= {{cname|upper}}_ERR_OVERHEAT;
        }
        if (responseError & (1 << 3)) {
            {% if data.debug %}
            Serial.print("RANGE_LIMIT ");
            {% endif %}
            err |= {{cname|upper}}_ERR_RANGE_LIMIT;
        }
        if (responseError & (1 << 4)) {
            {% if data.debug %}
            Serial.print("BAD_CHECKSUM ");
            {% endif %}
            err |= {{cname|upper}}_ERR_BAD_CHECKSUM;
        }
        if (responseError & (1 << 5)) {
            {% if data.debug %}
            Serial.print("OVERLOAD ");
            {% endif %}
            err |= {{cname|upper}}_ERR_OVERLOAD;
        }
        if (responseError & (1 << 6)) {
            {% if data.debug %}
            Serial.print("INSTRUCTION ");
            {% endif %}
            err |= {{cname|upper}}_ERR_INSTRUCTION;
        }

        {% if data.debug %}
        Serial.println("|");
        {% endif %}
    }

    return err;
}

int {{cname}}::read(unsigned char adr, unsigned char len, unsigned char *data) {
    while (true) {
        unsigned char b[2];
        startPacket({{cname|upper}}_READ_DATA);
        b[0] = adr;
        b[1] = len;
        bufferParams(b, 2);
        sendPacket();

        // Get the response
        int err = 0;
        err = doReceive({{cname|upper}}_RX_TIMEOUT);

        if (err == {{cname|upper}}_ERR_CORRUPTION) {
            // Packet got corrupted, try again
            continue;
        }

        if (!(err & {{cname|upper}}_ERR_TIMEOUT) && _responseParams == len) {
            for (int i = 0; i < len; i++) {
                data[i] = _responseData[i];
            }
        }

        return err;
    }
}

int {{cname}}::write(unsigned char adr, unsigned char len, unsigned char *data) {
    while (true) {
        unsigned char b[1];
        startPacket({{cname|upper}}_WRITE_DATA);
        b[0] = adr;
        bufferParams(b, 1);
        bufferParams(data, len);
        sendPacket();

        // Low return level
        return 0;
        /*int err = doReceive({{cname|upper}}_RX_TIMEOUT);
        if (err == {{cname|upper}}_ERR_CORRUPTION) {
            // Packet got corrupted, try again
            continue;
        }
        return err;*/
    }
}
