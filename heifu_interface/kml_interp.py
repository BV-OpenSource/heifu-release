from os import path
import shutil
import numpy as np
from scipy import signal
from pykml import parser
from lxml import etree

file_path = path.join('itaNFZForbiddenOnly_N.kml')
#fileN_path = path.join('portugalN.kml')
#shutil.copyfile(file_path, fileN_path)

file=open(file_path)
#fileN=open(fileN_path)

tree = etree.parse(file)
root = tree.getroot()
#treeN = etree.parse(fileN)
#rootN = treeN.getroot()
def resampleCoord(array, coordinates):
    sampleRatio = 64
    i = 0
    j = 0
    arLen = int(len(array))
    if (arLen < 30):
        array2 = np.empty((arLen * sampleRatio - sampleRatio + 1, 2))
        while i < 2:
            j = 0
            while j < arLen - 1:
                array2[(sampleRatio * j):(sampleRatio * j + sampleRatio):1, i] = signal.resample([array[j][i], array[j + 1][i]], sampleRatio*2)[0:sampleRatio:1]
                j += 1
            array2[arLen * sampleRatio - sampleRatio, i] = array[0][i]
            i += 1
        newCord = []
        [newCord.append(str(list(i)).strip("[]").replace(" ", "")) for i in array2[0:arLen * 3 * sampleRatio:1, 0:3:1]]
        newCordText = " ".join(newCord)
        coordinates.text = newCordText
        print("array")


for Document in root:
    for Folder in Document:
        for Placemark in Folder.findall('{http://www.opengis.net/kml/2.2}Placemark'):
            for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                    for outerBoundaryIs in Polygon.findall('{http://www.opengis.net/kml/2.2}outerBoundaryIs'):
                        for LinearRing in outerBoundaryIs:  # don't use Findall here because only 1 subelement
                            for coordinates in LinearRing:
                                coordinates_text_before = repr(coordinates.text)
                                coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                                array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
                                resampleCoord(array, coordinates)
        for Folder2 in Folder:
            for Placemark in Folder2.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                    for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                        for outerBoundaryIs in Polygon.findall('{http://www.opengis.net/kml/2.2}outerBoundaryIs'):
                            for LinearRing in outerBoundaryIs:  # don't use Findall here because only 1 subelement
                                for coordinates in LinearRing:
                                    coordinates_text_before = repr(coordinates.text)
<<<<<<< HEAD
                                    coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                                    array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
=======
                                    coordinates_split_before = coordinates_text_before.replace("'", "").replace("\\", "").replace("t", "").replace("n","").split(" ")
                                    array = [np.array(list(map(float, i))) for i in [i.split(",") for i in (coordinates_split_before)][:-1]]
                                    resampleCoord(array, coordinates)
            for Folder3 in Folder2:
                for Placemark in Folder3.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                    for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                                        coordinates_text_before = repr(coordinates.text)
<<<<<<< HEAD
                                        coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                                        array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
=======
                                        coordinates_split_before = coordinates_text_before.replace("'", "").replace("\\", "").replace("t", "").replace("n","").split(" ")
                                        array = [np.array(list(map(float, i))) for i in [i.split(",") for i in (coordinates_split_before)][:-1]]
>>>>>>> f_temp_OCT_22
                                        resampleCoord(array, coordinates)
                for Folder4 in Folder3:
                    for Placemark in Folder4.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                        for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                            for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                for outerBoundaryIs in Polygon.findall('{http://www.opengis.net/kml/2.2}outerBoundaryIs'):
                                    for LinearRing in outerBoundaryIs:  # don't use Findall here because only 1 subelement
                                        for coordinates in LinearRing:
                                            coordinates_text_before = repr(coordinates.text)
<<<<<<< HEAD
                                            coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                                            array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
=======
                                            coordinates_split_before = coordinates_text_before.replace("'", "").replace("\\", "").replace("t", "").replace("n","").split(" ")
                                            array = [np.array(list(map(float, i))) for i in [i.split(",") for i in (coordinates_split_before)][:-1]]
>>>>>>> f_temp_OCT_22
                                            resampleCoord(array, coordinates)
                    for Folder5 in Folder4:
                        for Placemark in Folder5.findall('{http://www.opengis.net/kml/2.2}Placemark'):
                            for MultiGeometry in Placemark.findall('{http://www.opengis.net/kml/2.2}MultiGeometry'):
                                for Polygon in MultiGeometry.findall('{http://www.opengis.net/kml/2.2}Polygon'):
                                    for outerBoundaryIs in Polygon.findall('{http://www.opengis.net/kml/2.2}outerBoundaryIs'):
                                        for LinearRing in outerBoundaryIs:  # don't use Findall here because only 1 subelement
                                            for coordinates in LinearRing:
                                                coordinates_text_before = repr(coordinates.text)
<<<<<<< HEAD
                                                coordinates_split_before = coordinates_text_before.replace("\\n", " ").replace("'", "").replace("\\",                                                                                                       "").replace(
                    "t", "").replace("n", "").split(" ")
                                                array = [np.array(list(map(float, i))) for i in
                        [i.split(",") for i in (coordinates_split_before) if i != "" ][:-1]]
                                                resampleCoord(array, coordinates)
treeN = etree.ElementTree(root)
treeN.write('itaNFZForbiddenOnly_N2.kml')
=======
                                                coordinates_split_before = coordinates_text_before.replace("'", "").replace("\\", "").replace("t", "").replace("n","").split(" ")
                                                array = [np.array(list(map(float, i))) for i in [i.split(",") for i in (coordinates_split_before)][:-1]]
                                                resampleCoord(array, coordinates)
treeN = etree.ElementTree(root)
treeN.write('ptNFZ_N.kml')
>>>>>>> f_temp_OCT_22
