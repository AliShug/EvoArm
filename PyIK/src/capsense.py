from collections import deque

import serial

WINDOW_SIZE = 512

class CapacitiveSensor:
    def __init__(self, port):
        self.serial = serial.Serial(port, 250000)
        self.readings = deque([])

    def updateReadings(self):
        while self.serial.in_waiting > 0:
            try:
                val = float(self.serial.readline())
            except ValueError:
                pass
            else:
                if len(self.readings) < WINDOW_SIZE:
                    self.readings.append(val)
                else:
                    self.readings.popleft()
                    self.readings.append(val)

    def latest(self):
        if len(self.readings) == 0:
            return None
        else:
            return self.readings[-1]

    def mean(self, n = WINDOW_SIZE):
        sum = 0.0
        count = min(len(self.readings), n)
        for i in xrange(count):
            sum += self.readings[-i]
        return sum / count
