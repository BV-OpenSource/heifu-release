#!/usr/bin/env python

import mavros_msgs, geometry_msgs.msg, geographic_msgs.msg
from mavros_msgs.msg import CommandCode
import time, json
import std_msgs.msg

class Mission:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.wpList = []
        self.total_waypoints_mission = 0
        self.waypoint_counter = 0

    def WaypointCv(self, msg):
        conversion = mavros_msgs.msg.Waypoint()
        conversion.frame = 3
        conversion.command = int(msg['command'])
        conversion.is_current = False
        conversion.autocontinue = True
        conversion.param1 = float(msg['param1'])
        conversion.param2 = float(msg['param2'])
        conversion.param3 = float(msg['param3'])
        conversion.param4 = float(msg['param4'])
        conversion.x_lat = float(msg['xLat'])
        conversion.y_long = float(msg['yLong'])
        conversion.z_alt = float(msg['zAlt'])

        return conversion, conversion.command

    def onGetMission(self, msg):
        print("Mission Pull requested from beXStream")
        wp = []
        for waypoint in self.wpList.waypoints:
            jsonObject = json.dumps({
                "command": waypoint.command,
                "param1":  waypoint.param1,
                "param2":  waypoint.param2,
                "param3":  waypoint.param3,
                "param4":  waypoint.param4,
                "x_lat":   waypoint.x_lat,
                "y_long":  waypoint.y_long,
                "z_alt":   waypoint.z_alt}, sort_keys=True)
            wp.append(jsonObject)

        self.vehicle.socket.emit('/mission/droneMission', wp)

    def onPush(self, msg):
        self.vehicle.pubStopMission.publish()

        if (self.vehicle._state.mode == '' or self.vehicle._state.mode == "AUTO" or self.vehicle._state.mode == "GUIDED"):
            self.vehicle.pubLoiter.publish() # Change to armable mode and assure that the vehicle don't fall if flying

            cntLoiter = 0
            while(self.vehicle._state.mode != "LOITER"):
                time.sleep(0.1)
                cntLoiter += 1

                if (cntLoiter == 20):
                    self.vehicle.logger.error('Unable to change to LOITER mode')
                    return

        # time.sleep(3)  # Wait for takeoff 3 seconds
        result = self.vehicle.srvMissionClear()
        self.vehicle.logger.info('Clear Mission-> %s' % str(result))
        waypoints_code = []
        self.waypoint_counter = 0
        waypointList = mavros_msgs.msg.WaypointList()

        # Adding global waypoint (probably to set altitude);
        initialWaypoint = mavros_msgs.msg.Waypoint()
        initialWaypoint.frame = 0  # global
        initialWaypoint.command = CommandCode.NAV_WAYPOINT  # waypoint
        initialWaypoint.is_current = False
        initialWaypoint.autocontinue = True
        initialWaypoint.param1 = 0.0
        initialWaypoint.param2 = 0.0
        initialWaypoint.param3 = 0.0
        initialWaypoint.param4 = 0.0
        initialWaypoint.x_lat = self.vehicle.uav_position.latitude
        initialWaypoint.y_long = self.vehicle.uav_position.longitude
        print("ALTITUDE: ",self.vehicle.uav_position.altitude);
        initialWaypoint.z_alt = self.vehicle.uav_position.altitude
        waypointList.waypoints.append(initialWaypoint)

        i = 0
        totalMissionSize = len(msg)

        while i < totalMissionSize:
            conversionWP, codeWP = self.WaypointCv(msg[i])

            waypointList.waypoints.append(conversionWP)

            # only save codes different than camera trigger code
            if codeWP == CommandCode.NAV_WAYPOINT or codeWP == CommandCode.NAV_TAKEOFF or codeWP == CommandCode.NAV_LAND:
                waypoints_code.append(codeWP)
            i += 1
        self.total_waypoints_mission = len(waypoints_code)
        self.vehicle.logger.info('Mission Push')
        self.vehicle.logger.info('%s' % str(waypointList))

        self.vehicle._state.onMission = False
        self.vehicle.srvMissionPush(0, waypointList.waypoints)
        self.vehicle.socket.emit('/mission/pushACK', {'mission': msg})

    # This function start mission on drone.
    # Drone must be in GUIDED MODE and Armed to start a mission
    def doMission(self, msg):
        print("DO MISSION")
        if self.vehicle.no_Takeoff == False:
            self.vehicle.logger.info('/mission/start %s' % str(msg))
#            try:
#                # Seting in GUIDED MODE
#                # self.vehicle.srvSetMode(0, "GUIDED")
##                self.vehicle.pubGuided.publish()
#                # commandLong "(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)"
#                # Arming Drone -- Command 400, confirmation 1, param1 (1 to arm, 0 to disarm)
##                self.vehicle.srvCmd(0, 400, 1, 1, 0, 0, 0, 0, 0, 0)

##                self.vehicle.pubArm.publish()
#                pass
#            except Exception as e:
#                self.vehicle.logger.error('Error on Arming Drone: ' + str(e))
#                pass
#            time.sleep(2)  # Wait for arm

            if (self.vehicle._state.isLanded):
                if (self.vehicle._state.mode == '' or self.vehicle._state.mode == "AUTO" or self.vehicle._state.mode == "GUIDED"):
                    self.vehicle.pubLoiter.publish() # Change to armable mode

                    cntLoiter = 0
                    while(self.vehicle._state.mode != "LOITER"):
                        time.sleep(0.1)
                        cntLoiter += 1

                        if (cntLoiter == 20):
                            self.vehicle.logger.error('Unable to change to LOITER mode')
                            return

                cntTakeoff = 0
                takeoffAlt = std_msgs.msg.UInt8()
                takeoffAlt.data = 1
                self.vehicle.pubTakeoff.publish(takeoffAlt)
                if (self.vehicle._state.isLanded):
                    time.sleep(0.1)
                    cntTakeoff += 1

                    if (cntTakeoff == 20):
                        self.vehicle.logger.error('Unable to TAKEOFF')
                        return

            self.vehicle.socket.emit('/mission/takeoffACK', {'msg': True})

            self.vehicle.pubStartMission.publish()
            self.vehicle._state.onMission = True
            self.vehicle.socket.emit('/mission/startACK', '')



#            try:
#                # commandLong "(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)"
#                # Sending start mission -- Command 300, confirmation 1, params all 0
##                self.vehicle.srvCmd(0, 300, 1, 0, 0, 0, 0, 0, 0, 0)
#                self.vehicle.pubAuto.publish()

#                self.vehicle.socket.emit('/mission/takeoffACK', True)
#                self.vehicle.socket.emit('/mission/startACK', '')
#                pass
#            except Exception as e:
#                self.vehicle.logger.info('Error on Start Mission: ' + str(e))
#                pass

    def onMissionStart(self, msg):
        print('STARTING MISSION')

        if self.vehicle._state.onMission:
            return

        self.doMission(msg)

    def onMissionCancel(self, msg):
        self.vehicle.logger.info('/mission/stop')
        self.vehicle.pubStopMission.publish()
        self.vehicle.socket.emit('/mission/stopACK', '')
        self.vehicle._state.onMission = False

    def onMissionResume(self, msg):
        self.vehicle.logger.info('/mission/resume')
        self.vehicle.pubAuto.publish()
        self.vehicle._state.onMission = True
        self.vehicle.socket.emit('/mission/resumeACK', '')

    def onMissionClear(self, msg):
        self.vehicle.logger.info('/mission/clear')
        result = self.vehicle.srvMissionClear()
        self.vehicle.logger.info('Clear Mission-> %s' % str(result))
        self.vehicle.socket.emit('/mission/clearACK', result.success)


    def cbWaypointList(self, msg):
        self.wpList = mavros_msgs.msg.WaypointList()
        self.wpList = msg
        self.total_waypoints_mission = len(self.wpList.waypoints) - 1

    def cbWaypointReached(self, msg):
        if self.vehicle._state.onMission:
            self.waypoint_counter = msg.wp_seq # Begins in 0
            percentage = 100.0 * \
                (float(self.waypoint_counter) / float(self.total_waypoints_mission))

            self.vehicle.socket.emit('/mission/status',
                                     ({'total': self.total_waypoints_mission, 'reached': self.waypoint_counter, 'percentage': percentage}))
            self.vehicle.logger.info('Total-> %s || Reached-> %s || Percentage-> %s' % (
                str(self.total_waypoints_mission), str(self.waypoint_counter), str(percentage)))

            if int(percentage) >= int(100):
                self.vehicle.logger.info('End Mission')
                self.vehicle.socket.emit('/mission/landACK', True)
                self.vehicle.socket.emit('/mission/end', '')
                self.vehicle._state.onMission = False

class Setpoint:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def onSetpoint(self, msg):
        self.vehicle.logger.info('SetPoint')
        waypointPosition = geographic_msgs.msg.GeoPoint()
        waypointPosition.latitude = msg['latitude']
        waypointPosition.altitude = msg['altitude']
        waypointPosition.longitude = msg['longitude']

        self.vehicle.pubSetPoint.publish(waypointPosition)
