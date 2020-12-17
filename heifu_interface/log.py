#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: João Pedro Carvalho
"""

import sys
import datetime
import os
from os.path import expanduser
from time import sleep

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
DEFAULT = '\033[39m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'



# logging has too many quirks...
class HEIFULogger:
    def __init__(self, filePythonName='heifu_interface', writeFile=False):
        self.writeFile = writeFile
        self.show_debug = False
        self.filePythonName = filePythonName
        self.path = None
        self.ticketToWrite = True
        if self.writeFile:
            self.date = datetime.datetime.now()
            homeDir = expanduser("~")
            fileName = str(self.filePythonName) + '-' + self.date.strftime("%Y-%m-%d&%H:%M:%S") + '.txt'
            folderPath = os.path.abspath(os.path.join(homeDir, 'Desktop', 'logs'))
            self.path = os.path.abspath(os.path.join(folderPath, fileName))
            if not os.path.exists(folderPath):  # Verify if the folder exist
                os.mkdir(folderPath)

    def log(self, startc, msg, level_name):
        while self.ticketToWrite == False:
            sleep(0.01)
            continue
        self.ticketToWrite = False
        
        self.date = datetime.datetime.now()
        level = ('[' + level_name + ']').ljust(9)
        print('%s %s%s %s%s' % (self.date.strftime("%Y-%m-%d %H:%M:%S"), startc, level, msg, ENDC))
        if self.writeFile:
            self.f = open(self.path, 'a')
            self.f.write('%s %s %s\n' % (self.date.strftime('%Y-%m-%d %H:%M:%S'), level, msg))
            self.f.close()
        sys.stdout.flush()
        
        self.ticketToWrite = True

    def info(self, msg):
        self.log(DEFAULT, msg, 'INFO')

    def warning(self, msg):
        self.log(WARNING, msg, 'WARNING')

    def error(self, msg):
        self.log(FAIL, msg, 'ERROR')

    def exception(self, msg):
        self.log(FAIL, msg, 'EXCEPTION')

    def debug(self, msg):
        if self.show_debug:
            self.log(OKGREEN, msg, 'DEBUG')

