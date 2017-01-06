import serial
import sys
import threading
import binascii
import time

quit = False
ser = serial.Serial('COM3', 1000000)

send_ready = True

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

def sender():
    global quit
    global send_ready

    while True:
        data = ''.join(raw_input().split())

        if data == 'q':
            quit = True
            break
        else:
            packet = genPacket(data)
            if packet is not None:
                ser.write(packet)
                # get response
                while ser.in_waiting < 1:
                    pass
                print('Response >> {0}'.format(
                    binascii.hexlify(ser.read_all())))

def receiver():
    global send_ready

    while True:
        if quit:
            break
        elif ser.in_waiting > 1:
            byte = ser.read()
            if byte == '\xFF':
                send_ready = True
            sys.stdout.write(hex(ord(byte)))
            sys.stdout.write(' ')

sender()
#receiver()
