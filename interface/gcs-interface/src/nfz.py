#!/usr/bin/env python

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import formulas
import nfz_stop
from time import sleep
from threading import Thread, Timer

class NFZ:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.nfz_complete = False
        self.areaIndex = 0
        self.threshold_min_areas = []
        self.threshold_min_distances = []
        self.location_array = []

    def calculatePolygon(self, takeoffFlag):
        if self.threshold_min_areas:
            if takeoffFlag:
                polygon = []
                for area in self.threshold_min_areas:
                    listPoints = []
                    for coordinate in area:
                        listPoints.append(Point(coordinate[1], coordinate[0]))
                    polygon.append(Polygon(listPoints))
            else:
                listPoints = [Point(coordinate[1], coordinate[0])
                              for coordinate in self.threshold_min_areas[self.areaIndex]]
                polygon = Polygon(listPoints)
            return polygon

    def calculateAzimuthAndDistances(self, uav_position):
        if(self.threshold_min_areas):
            closestCoordinates = nfz_stop.getClosestCoordinateFromArea(
                uav_position.latitude, uav_position.longitude, self.threshold_min_areas)
            closestCoordinate = closestCoordinates[0]
            secondClosestCoordinate = closestCoordinates[1]
            self.areaIndex = closestCoordinates[2]
            coord = formulas.pnt2line([uav_position.latitude, uav_position.longitude, 0], [
                                      closestCoordinate[1], closestCoordinate[0], 0], [secondClosestCoordinate[1], secondClosestCoordinate[0], 0])
            # distance comes in degrees from pnt2line formula and one degree is 111km therefore distance is 111k meters * degree
            distance = 111000 * coord[0]
            if distance < 1000:
                self.vehicle.socket.emit('/distanceWarning', distance)

    def calculatethresholdMinDistances(self):
        while True:
            sleep(1)
            if self.vehicle.uav_position.longitude is not None and self.vehicle.uav_position.latitude is not None:
                threshold_area_distance = nfz_stop.thresholdClosestAreas(
                    self.vehicle.uav_position, self.location_array)
                self.threshold_min_areas = threshold_area_distance[0]
                self.threshold_min_distances = threshold_area_distance[1]

    def isInsideNoFlyZone(self):
        while True:
            # print("Calculate NFZ")

            sleep(1)
            if not self.vehicle._state.isLanded:
                if self.vehicle.uav_position.latitude is not None and self.vehicle.uav_position.longitude is not None:
                    point = Point(self.vehicle.uav_position.latitude,
                                self.vehicle.uav_position.longitude)
                    if self.vehicle._state.onManual == True:
                        polygons = self.calculatePolygon(True)
                        if polygons:
                            for polygon in polygons:
                                if polygon.contains(point) == True:
                                    self.vehicle.socket.emit(
                                        '/takeoffError', {'msg': True})
                                    self.vehicle.no_Takeoff = True
                                    break
                                else:
                                    self.vehicle.no_Takeoff = False
                    else:
                        polygon = self.calculatePolygon(False)
                        if polygon:
                            if polygon.contains(point):
                                self.vehicle.socket.emit(
                                    '/takeoffError', {'msg': True})
                                self.vehicle.no_Takeoff = True
                            else:
                                self.vehicle.no_Takeoff = False

                if self.vehicle.no_Takeoff == True and not self.vehicle._state.isLanded:
                    self.vehicle.pubLand.publish()

    def setupNFZ(self):
        if self.vehicle.enable_nfz:
            self.nfz_complete = True
            self.location_array = nfz_stop.seedCoordinates(
                self.vehicle.uav_position.latitude, self.vehicle.uav_position.longitude)
            self.thread_min_distances = Thread(
                target=self.calculatethresholdMinDistances)
            self.thread_min_distances.start()
            thread_inside_nfz = Thread(target=self.isInsideNoFlyZone)
            thread_inside_nfz.daemon = True
            thread_inside_nfz.start()