#!/usr/bin/env python3

import serial
import time
import rospy
import std_msgs
from rfid_rise.msg import rfid_humtemp
import binascii
import struct

readOut = 0 
commandStart = "START\n"  # Start sensor
commandStop = "CANCEL\n"  # Stop Sensor
commandReset = "SYSRST\n" # Reset the Raspberry system
global sending
global stopSuccess
stopSuccess = True

try:
  ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)
  print("Serial COM opened")
except Exception as e:
  print("Exception: Opening serial port: " + str(e))
  exit(1)


def trigger(msg):
  rise_sensor = rfid_humtemp()
  global stopSuccess
  global sending
  sending = True
  print("Reading sensors")
  # Reading sensor
  while sending and stopSuccess:
    dataHexRaw = ""
    tagId = ""
    reading = True
    ser.flush()  # flush the buffer
    ser.write(str(commandStart).encode()) #Start command
    time.sleep(.1)
    try:
      while reading:
        if sending is False:
          break
          pass
        readOut = ser.readline()
        # print(len(readOut.rstrip()))
        print(readOut)
        if len(readOut.rstrip()) > 35:
          reading = False
          pass
        pass
      #print(readOut)
      readOut = readOut.rstrip()
      data = readOut.split(',')
      print(data)
      dataHexRaw = data[1]
      dataHex = [ dataHexRaw[i:i+(len(dataHexRaw)/2)] for i in range(0, len(dataHexRaw), len(dataHexRaw)/2) ]
      print(dataHex)

      # Getting tag ID Sensor
      tagId = data[0]
      print(tagId)
    except:
      print("ERROR to get sensor DATA")
      pass


    # Decoding the little endian HEX to readable value
    try:
      hum = decodeData(dataHex[0])
      temp = decodeData(dataHex[1])

      rise_sensor.tag_id = tagId
      rise_sensor.humidity = float(hum)
      rise_sensor.temperature = float(temp)
      pubRfidSensor.publish(rise_sensor)
      sending = False
      pass
    except:
      print("ERROR to get sensor value")
      rise_sensor.tag_id = tagId
      rise_sensor.humidity = 0.0
      rise_sensor.temperature = 0.0
      pubRfidSensor.publish(rise_sensor)
      pass
    sending = False
    ser.flush()  # flush the buffer
    pass
        
  print("Stop reading")
  ser.flush()  # flush the buffer

# Decoder function
def decodeData(rawData):
  bigEndianData = []
  binData = ""
  decData = ""

  # Converts littleEndian data to bigEndian data
  for i in range(0, len(rawData), 2):
    bigEndianData.insert(0, rawData[i:i+2])
    pass

  # Converts Hex to Bin IEEE 754 Floating Point format
  for i in bigEndianData:
    binData += str("{0:08b}".format(int(i, 16)))
    pass

  # Convert Bin IEEE 754 Floating Point to Float readable value
  decData = binaryToFloat(binData)

  return decData

def stopSending(msg):
  global sending
  global stopSuccess
  stopSuccess = False
  sending = False
  ser.flush()  # flush the buffer
  ser.write(str(commandStop).encode()) #Stop command
  try:
    print("Canceling...")
    while stopSuccess is False:
      readOut = ser.readline()
      # print(len(readOut.rstrip()))
      print(readOut)
      if readOut.rstrip() == "CANCELED":
        stopSuccess = True
        pass
      pass
  except:
    print("ERROR to CANCEL")
    pass

def resetRasp(msg):
  reseting = True
  #stopSending("")
  ser.flush()  # flush the buffer
  ser.write(str(commandReset).encode()) #Stop command
  try:
    print("Reseting the system...")
    while reseting:
      readOut = ser.readline()
      # print(len(readOut.rstrip()))
      print(readOut)
      if readOut.rstrip() == "SUCCESS: Reader initialized.":
        reseting = False
        pass
      pass
  except:
    print("ERROR to RESETING SYSTEM")
    pass
  ser.flush()  # flush the buffer

# IEEE 754 convertion function
def binaryToFloat(value):
  hx = hex(int(value, 2))   
  return struct.unpack("f", struct.pack("I", int(hx, 16)))[0]

if __name__ == '__main__':
  try:
    print("Node Started")
    sending = False
    rospy.init_node('rfid_rise_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pubRfidSensor = rospy.Publisher('sensors/data', rfid_humtemp, queue_size=10)
    rospy.Subscriber("sensors/start", std_msgs.msg.Empty, trigger)
    rospy.Subscriber("sensors/cancel", std_msgs.msg.Empty, stopSending)
    rospy.Subscriber("sensors/reset", std_msgs.msg.Empty, resetRasp)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass