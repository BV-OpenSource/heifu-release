#!/usr/bin/env python

#from sensors_node.msg import rfid_humtemp

class RFID:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def cbSensorRfid(self, msg):
        sensorData = rfid_humtemp()
        sensorData = msg
        self.vehicle.logger.info(sensorData.humidity)
        self.vehicle.socket.emit('/mission/sensors/rfid', {
                                 'tagId': sensorData.tag_id, 'value': sensorData.temperature, 'humidity': sensorData.humidity})
        self.vehicle.pubSensorStop.publish()

    def sendTriggerRISESensor(self, msg):
        self.vehicle.logger.info('Trigger sensor')
        self.vehicle.pubSensorStart.publish()