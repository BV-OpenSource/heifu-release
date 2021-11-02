#!/usr/bin/env python

import geometry_msgs
import gimbal
from threading import Timer

GIMBAL_MIN_DELAY = 0.1  # 1 / GIMBAL_HZ

class Gimbal:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.gimbal_flag = True

    def TwistCv(self, msg):
        conversion = geometry_msgs.msg.Twist()
        #sent = json.loads(json.dumps(msg))
        conversion.linear.x = msg['x']
        conversion.linear.y = msg['Y']
        if msg['z'] < 0:
            conversion.linear.z = -1
            pass
        else:
            conversion.linear.z = msg['z']
            pass
        conversion.angular.x = msg['R']
        conversion.angular.y = msg['P']
        conversion.angular.z = msg['y'] / 6.0
        return conversion

    def unlock_gimbal_flag(self):
        # global gimbal_flag
        self.gimbal_flag = True

    def onGimbal(self, msg):
        self.vehicle.logger.info('Gimbal %s' % str(msg))
        gimbalControl = gimbal.msg.setGimbalAxes()
        gimbalControl.pitch = -msg['y']
        gimbalControl.roll = -msg['x']
        gimbalControl.yaw = 0
        try:
            # global gimbal_flags
            if self.gimbal_flag:
                Timer(GIMBAL_MIN_DELAY, self.unlock_gimbal_flag, ()).start()
                self.gimbal_flag = False
                pass
            self.vehicle.pubGimbal.publish(gimbalControl)
        except Exception as e:
            self.vehicle.logger.info('Error onGimbal callback' + str(e))

    def onCmd(self, msg):
        conversion = self.TwistCv(msg)
        self.vehicle.pubCmd.publish(conversion)
