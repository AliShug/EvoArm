import serial
import sys
import threading
import binascii
import time

ser = serial.Serial('COM3', 250000)

def checksum(bytes):
    sum = 0
    for b in bytes:
        sum += ord(b)
    return chr((~sum) & 0xFF)

def genPacket(data):
    try:
        bytes = binascii.unhexlify(data)
        num = len(bytes)+3
        bytes = chr(0) + chr(num) + '\xFF\xFF' + bytes + checksum(bytes)
        return bytes
    except:
        print('Bad input {0}'.format(data))
        return None

def send():
    packet = genPacket('020201')
    ser.write(packet)
    # get response
    #print('Response >> {0}'.format(binascii.hexlify(receive())))

def receive():
    while ser.in_waiting < 1:
        pass
    return ser.read_all()

time.sleep(2)

timer = time.clock()
for i in xrange(100):
    send()
    receive()
print('{0:.3f}s'.format(time.clock()-timer))
