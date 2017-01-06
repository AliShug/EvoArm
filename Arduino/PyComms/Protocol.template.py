# Auto-generated file
# Modify via protocol.yaml and/or Protocol.template.py
from __future__ import print_function

import struct
import time
import serial as pyserial

waitTime = 0.01

# Buffering system for bulk command issue
buffering = False

class TimeoutException(Exception):
    pass

class ArduinoConfigException(Exception):
    pass

# Connection and utility functions
def serialConnect(port, baud):
    """Creates, checks and returns a serial connection to a configured Arduino
    """
    serial = pyserial.Serial(port, baud)
    print ("Waiting for board on {0}".format(port), end=" ")
    serial.timeout = 5
    result = ''
    attempts = 1
    while result == '':
        print ("Attempt {0}".format(attempts))
        serial.close()
        serial.open()
        result = serial.readline()
        attempts += 1

    if not result.startswith("CommTest READY"):
        raise ArduinoConfigException("ERROR: PC/Board software mismatch", result)
    return serial

def findServos(serial):
    """When passed a serial connection to a configured Arduino, returns all
    connected servos.
    """
    servos = {'v1': [], 'v2': []}
    # Retrieves a list of servos over serial, one per line
    if serial is not None:
        timer = time.clock()
        # Set relatively high timeout for this section of board comms
        serial.timeout = 3
        command = "list"
        l = struct.pack('b', len(command))
        serial.write(l + command)
        read = serial.readline()
        x1num = int(read.split('=')[1])
        for i in range(x1num):
            str = serial.readline()
            if str.startswith("ERROR"):
                print ("Hardware Error during V1 servo listing: ", str)
            else:
                id = int(str)
                servos['v1'].append(Servo(serial, 1, id))
        read = serial.readline()
        x2num = int(read.split('=')[1])
        for i in range(x2num):
            str = serial.readline()
            if str.startswith("ERROR"):
                print ("Hardware Error during V2 servo listing: ", str)
            else:
                id = int(str)
                servos['v2'].append(Servo(serial, 2, id))
        # Revert to shorter timeout
        serial.timeout = 0.01
    # Return our output servo dictionary
    return servos



def waitFor(serial, num_bytes, timeout=0.1):
    start = time.time()

    # Busy wait until required bytes arrive or we timeout
    while time.time() < start + timeout:
        if serial.in_waiting >= num_bytes:
            return

    raise TimeoutException('Timeout')

def tryRead(serial, num_bytes, timeout=0.05):
    start = time.time()
    read_bytes = 0

    # Busy wait until required bytes arrive or we timeout
    while time.time() < start + timeout:
        if serial.in_waiting >= num_bytes:
            return serial.read(num_bytes)

    raise TimeoutException('Timeout')


class CapacitiveSensor:
    def __init__(self, serial):
        self.serial = serial

    def read(self, count):
        command = 'c{0}'.format(chr(count))
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, count*4)

        values = [0]*count
        for i in xrange(count):
            values[i] = struct.unpack('i', self.serial.read(4))[0]
        return values

class Servo:
    def __init__(self, serial, protocol_ver, id):
        self.protocol = protocol_ver
        self.id = id
        self.serial = serial
        self.data = {'pos': 150}

    # Templated commands
{% for c in commands if c.can_set %}
    def set{{c.name}}(self, val):
        command = 's{{"\\x{0:02X}".format(ord(c.short))}}{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            {% if c.type == 'int' or c.type == 'bool' %}
            arg = struct.pack('i', val)
            {% else %}
            arg = struct.pack('f', val)
            {% endif %}
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in get{{c.name}} <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in set{{c.name}} <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
{% endfor %}
{% for c in commands if c.can_get %}
    def get{{c.name}}(self):
        command = 'g{{"\\x{0:02X}".format(ord(c.short))}}{pver}{packedid}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 5)

        # Retreive response
        try:
            arg = tryRead(self.serial, 1)
            if arg != 'k':
                print ('Servo Error in get{{c.name}} <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            {% if c.type == 'int' or c.type == 'bool' %}
            val = struct.unpack('i', arg)[0]
            {% else %}
            val = struct.unpack('f', arg)[0]
            {% endif %}
            {% if c.type =='bool' %}
            return bool(val)
            {% else %}
            return val
            {% endif %}
        except Exception as e:
            print ('Bad receive in get{{c.name}} <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
{% endfor %}
#def getServos():
