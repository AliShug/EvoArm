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

    #raise TimeoutException('Timeout')

def tryRead(serial, num_bytes, timeout=0.05):
    start = time.time()
    read_bytes = 0

    # Busy wait until required bytes arrive or we timeout
    while time.time() < start + timeout:
        if serial.in_waiting >= num_bytes:
            return serial.read(num_bytes)

    #raise TimeoutException('Timeout')


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
    def setID(self, val):
        command = 's\x02{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getID <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setID <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setReturnDelay(self, val):
        command = 's\x03{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getReturnDelay <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setReturnDelay <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCWLimit(self, val):
        command = 's\x04{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCWLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCWLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCCWLimit(self, val):
        command = 's\x05{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCCWLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCCWLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setTempLimit(self, val):
        command = 's\x06{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getTempLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setTempLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setLowVoltageLimit(self, val):
        command = 's\x07{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getLowVoltageLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setLowVoltageLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setHighVoltageLimit(self, val):
        command = 's\x08{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getHighVoltageLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setHighVoltageLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setMaxTorque(self, val):
        command = 's\x09{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getMaxTorque <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setMaxTorque <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setMaxTorque(self, val):
        command = 's\x0A{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getMaxTorque <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setMaxTorque <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setAlarmFlags(self, val):
        command = 's\x0B{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getAlarmFlags <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setAlarmFlags <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setShutdownFlags(self, val):
        command = 's\x0C{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getShutdownFlags <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setShutdownFlags <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setTorqueEnable(self, val):
        command = 's\x0D{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getTorqueEnable <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setTorqueEnable <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setLED(self, val):
        command = 's\x0E{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getLED <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setLED <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setGoalPosition(self, val):
        command = 's\x14{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getGoalPosition <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setGoalPosition <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setGoalSpeed(self, val):
        command = 's\x15{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getGoalSpeed <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setGoalSpeed <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setTorqueLimit(self, val):
        command = 's\x16{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getTorqueLimit <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setTorqueLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCWMargin(self, val):
        command = 's\x17{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCWMargin <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCWMargin <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCCWMargin(self, val):
        command = 's\x18{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCCWMargin <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCCWMargin <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCWSlope(self, val):
        command = 's\x19{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCWSlope <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCWSlope <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setCCWSlope(self, val):
        command = 's\x1A{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('i', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getCCWSlope <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setCCWSlope <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def setPunch(self, val):
        command = 's\x1B{pver}{packedid}{arg}'.format(
            pver = self.protocol,
            packedid = struct.pack('B', self.id),
            arg = struct.pack('f', val)
        )
        l = struct.pack('b', len(command))
        self.serial.write(l+command)
        waitFor(self.serial, 2)

        # Response
        res = 'Timeout in getPunch <dx{0}:{1}>'.format(self.protocol, self.id)
        while self.serial.in_waiting > 0:
            res = self.serial.readline()
        if res.startswith('ERROR'):
            print ('Servo Error in setPunch <dx{0}:{1}> {2}'.format(self.protocol, self.id, res))
            return False
        return True
    def getModelNumber(self):
        command = 'g\x00{pver}{packedid}'.format(
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
                print ('Servo Error in getModelNumber <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getModelNumber <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getFirmwareVersion(self):
        command = 'g\x01{pver}{packedid}'.format(
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
                print ('Servo Error in getFirmwareVersion <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getFirmwareVersion <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getID(self):
        command = 'g\x02{pver}{packedid}'.format(
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
                print ('Servo Error in getID <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getID <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getReturnDelay(self):
        command = 'g\x03{pver}{packedid}'.format(
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
                print ('Servo Error in getReturnDelay <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getReturnDelay <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCWLimit(self):
        command = 'g\x04{pver}{packedid}'.format(
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
                print ('Servo Error in getCWLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCWLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCCWLimit(self):
        command = 'g\x05{pver}{packedid}'.format(
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
                print ('Servo Error in getCCWLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCCWLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getTempLimit(self):
        command = 'g\x06{pver}{packedid}'.format(
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
                print ('Servo Error in getTempLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getTempLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getLowVoltageLimit(self):
        command = 'g\x07{pver}{packedid}'.format(
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
                print ('Servo Error in getLowVoltageLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getLowVoltageLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getHighVoltageLimit(self):
        command = 'g\x08{pver}{packedid}'.format(
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
                print ('Servo Error in getHighVoltageLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getHighVoltageLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getMaxTorque(self):
        command = 'g\x09{pver}{packedid}'.format(
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
                print ('Servo Error in getMaxTorque <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getMaxTorque <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getMaxTorque(self):
        command = 'g\x0A{pver}{packedid}'.format(
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
                print ('Servo Error in getMaxTorque <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getMaxTorque <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getAlarmFlags(self):
        command = 'g\x0B{pver}{packedid}'.format(
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
                print ('Servo Error in getAlarmFlags <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getAlarmFlags <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getShutdownFlags(self):
        command = 'g\x0C{pver}{packedid}'.format(
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
                print ('Servo Error in getShutdownFlags <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getShutdownFlags <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getTorqueEnable(self):
        command = 'g\x0D{pver}{packedid}'.format(
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
                print ('Servo Error in getTorqueEnable <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return bool(val)
        except Exception as e:
            print ('Bad receive in getTorqueEnable <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getLED(self):
        command = 'g\x0E{pver}{packedid}'.format(
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
                print ('Servo Error in getLED <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getLED <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getVoltage(self):
        command = 'g\x0F{pver}{packedid}'.format(
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
                print ('Servo Error in getVoltage <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getVoltage <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getPosition(self):
        command = 'g\x10{pver}{packedid}'.format(
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
                print ('Servo Error in getPosition <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getPosition <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getLoad(self):
        command = 'g\x11{pver}{packedid}'.format(
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
                print ('Servo Error in getLoad <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getLoad <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getTemperature(self):
        command = 'g\x12{pver}{packedid}'.format(
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
                print ('Servo Error in getTemperature <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getTemperature <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getSpeed(self):
        command = 'g\x13{pver}{packedid}'.format(
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
                print ('Servo Error in getSpeed <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getSpeed <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getGoalPosition(self):
        command = 'g\x14{pver}{packedid}'.format(
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
                print ('Servo Error in getGoalPosition <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getGoalPosition <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getGoalSpeed(self):
        command = 'g\x15{pver}{packedid}'.format(
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
                print ('Servo Error in getGoalSpeed <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getGoalSpeed <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getTorqueLimit(self):
        command = 'g\x16{pver}{packedid}'.format(
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
                print ('Servo Error in getTorqueLimit <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getTorqueLimit <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCWMargin(self):
        command = 'g\x17{pver}{packedid}'.format(
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
                print ('Servo Error in getCWMargin <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCWMargin <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCCWMargin(self):
        command = 'g\x18{pver}{packedid}'.format(
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
                print ('Servo Error in getCCWMargin <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCCWMargin <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCWSlope(self):
        command = 'g\x19{pver}{packedid}'.format(
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
                print ('Servo Error in getCWSlope <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCWSlope <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getCCWSlope(self):
        command = 'g\x1A{pver}{packedid}'.format(
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
                print ('Servo Error in getCCWSlope <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getCCWSlope <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getPunch(self):
        command = 'g\x1B{pver}{packedid}'.format(
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
                print ('Servo Error in getPunch <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('f', arg)[0]
            return val
        except Exception as e:
            print ('Bad receive in getPunch <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
    def getMoving(self):
        command = 'g\x1C{pver}{packedid}'.format(
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
                print ('Servo Error in getMoving <dx{0}:{1}> E{2}'.format(self.protocol, self.id, self.serial.readline()))
                return None
            arg = tryRead(self.serial, 4)
            val = struct.unpack('i', arg)[0]
            return bool(val)
        except Exception as e:
            print ('Bad receive in getMoving <dx{0}:{1}> {2}'.format(self.protocol, self.id, e))
            return None
#def getServos():
