from os import path
import rospy
import shutil
import numpy as np
import sensor_msgs.msg
from lxml import etree
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import math
import formulas
import time
from threading import Thread
from time import sleep
import countries

PROJECTPATH = '/home/heifu/heifu_ws/src/heifu/heifu_interface/'

global root
location_array = []
def appendToArray(coordinateList):
    global location_array
    location_array.append(coordinateList)

def chooseKMLFile(lat,long):
    global root
    cc = countries.CountryChecker(PROJECTPATH + 'TM_WORLD_BORDERS-0.3.shp')
    country = cc.getCountry(countries.Point(lat, long)).iso
    if (country == 'IT'):
        file_path = path.join(PROJECTPATH + 'itaNFZForbiddenOnly_N.kml')
    elif (country == 'PT'):
        file_path = path.join(PROJECTPATH + 'ptNFZForbiddenOnly_N.kml')
    else:
        file_path = path.join(PROJECTPATH +'ptNFZForbiddenOnly_N.kml')
    print('Currently flying in:' + country)
    file=open(file_path)
    tree = etree.parse(file)
    root = tree.getroot()

def findAll(section):
    for outerBoundaryIs in section.findall('{http://www.opengis.net/kml/2.2}outerBoundaryIs'):
        for LinearRing in outerBoundaryIs:  # don't use Findall here because only 1 subelement
            for coordinates in LinearRing:
                coordinates_text_before = repr(coordinates.text)
                coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
                appendToArray(array)

def seedCoordinates(lat,long):
    chooseKMLFile(lat,long)
    global location_array
    for Document in root:
        for Folder in Document:
            for Placemark in Folder.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                for Polygon in Placemark.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                    findAll(Polygon)
                for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                    for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                        findAll(Polygon)
            for Folder2 in Folder:
                for Placemark in Folder2.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                    for Polygon in Placemark.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                        findAll(Polygon)
                    for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                        for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                            findAll(Polygon)
                for Folder3 in Folder2:
                    for Placemark in Folder3.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                        for Polygon in Placemark.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                            findAll(Polygon)
                        for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                            for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                findAll(Polygon)
                    for Folder4 in Folder3:
                        for Placemark in Folder4.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                            for Polygon in Placemark.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                findAll(Polygon)
                            for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                                for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                    findAll(Polygon)
                        for Folder5 in Folder4:
                            for Placemark in Folder5.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                                for Polygon in Placemark.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                    findAll(Polygon)
                                for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                                    for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                        findAll(Polygon)
    return location_array


####################################################### Calculations after getting all coordinates ###################################
def thresholdClosestAreas(UAVposition, zoneArray):
    thresholdMinDistances = []
    thresholdMinAreas = []
    thresholdMinAreaIndexes = []
    thresholdDistance = 100000
    minDistances = np.array([np.amin([formulas.distanceBetweenPoints(coordinate[1], coordinate[0], UAVposition.latitude ,UAVposition.longitude) for coordinate in location]) for location in location_array])
    thresholdMinAreaIndexes = np.where(np.array(minDistances < thresholdDistance ) == True)
    thresholdMinDistances = minDistances[thresholdMinAreaIndexes]
    [thresholdMinAreas.append(zoneArray[idx]) for idx in thresholdMinAreaIndexes[0]]
    return (thresholdMinAreas, thresholdMinDistances)

def getClosestCoordinateFromArea(lat, long, thresholdMinAreas):
    distanceArray = []
    if thresholdMinAreas:
        distanceArray = np.array([[formulas.distanceBetweenPoints(coordinate[1], coordinate[0], lat, long) for coordinate in coordinates] for coordinates in thresholdMinAreas])
        sortedDistances = [np.sort(np.array(zoneDistances).flatten()) for zoneDistances in distanceArray]
        minDistanceAreas = [np.amin(l) for l in sortedDistances]
        areaIndex = minDistanceAreas.index(min(minDistanceAreas))
        firstMinimumIndex = np.where( distanceArray[areaIndex] == sortedDistances[areaIndex][0])[0][0]
        secondMinimumIndex = np.where( distanceArray[areaIndex] == sortedDistances[areaIndex][1])[0][0]
        closestCoordinate = thresholdMinAreas[areaIndex][firstMinimumIndex]
        secondClosestCoordinate = thresholdMinAreas[areaIndex][secondMinimumIndex]
        return (closestCoordinate, secondClosestCoordinate, areaIndex)
