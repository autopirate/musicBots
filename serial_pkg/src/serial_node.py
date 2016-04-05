#!/usr/bin/env python

import serial
import numpy as np
import sys
import time
import rospy
import std_msgs.msg


class pubsub:
    def __init__(self,pub_name):
        self.irPub = rospy.Publisher(pub_name, std_msgs.msg.Float64, queue_size=10, latch=True)

    def publish_ir(self,reading):
        msg = std_msgs.msg.Float64()
        msg.data = reading
        self.irPub.publish(msg)
        return

class serialRead:
    def __init__(self, serialPort_, baudRate_):
        self.serialPort = serial.Serial()
        self.serialPort.port = serialPort_
        self.serialPort.baudrate = baudRate_
        self.serialPort.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.serialPort.parity = serial.PARITY_NONE  # set party check: no parity
        self.serialPort.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self.serialPort.timeout = 1  # non-block read
        self.serialPort.xonxoff = False  # disable software flow control
        self.serialPort.rtscts = False  # disable hardware (RTS/CTS) flow control
        self.serialPort.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self.serialPort.writeTimeout = 2  # timeout for write

    def readData(self):
        try:
            data_e = self.serialPort.readline()
            data_e = data_e.rstrip()
            return data_e
        except Exception, e:
            print "Error reading data:", str(e)
            return None


if __name__ == '__main__':
    try:
        #Initialize node
        rospy.init_node('serial_read_MIDI', anonymous=True)
        rate = rospy.Rate(50) #50Hz

        #Initialize Publisher
        pubsub_obj = pubsub('/serial/MIDI_feedback')

        #Initialize Serial Port
        ser2 = serialRead('/dev/ttyACM1', 9600)
        
        print "Serial_read: Set-Up Complete", "Reading Data in 5 Seconds"
        time.sleep(5)
        ser2.serialPort.open()

        if ser2.serialPort.isOpen():
            print "Port Open, Starting Reading Process"
            ser2.serialPort.flushInput()
            ser2.serialPort.flushOutput()

            while not rospy.is_shutdown(): #Read till ROS master kills node
                data_MIDI = ser2.readData()
                if data_IR.isdigit() == True:
                    data_MIDI = float(data_MIDI)
                    pubsub_obj.publish_ir(data_MIDI)
                rate.sleep()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin() #Node doesn't die