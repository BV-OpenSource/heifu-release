#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: João Pedro Carvalho
"""


import pyaudio  
import os
import wave  
import argparse

# Argument parser
parser = argparse.ArgumentParser(description='Play any wav audio.')
parser.add_argument("-b", "--base",    type=str,   default='/home/heifu/',         help='Base directory')
parser.add_argument("-d", "--dir",     type=str,   default='audios',     help='File directory')
parser.add_argument("-f", "--file",    type=str,   default='audio.wav', help='Input file.')
parser.add_argument(      "--loop",  action="store_true", help='Loop.')
parser.add_argument("-v", "--version",         help="show program version",              action="store_true")

args = parser.parse_args()
file = os.path.join(args.base, args.dir, args.file)

if args.version:
    print('Version 0.1.')
    exit()


#define stream chunk   
chunk = 1024  

#open a wav format music  
f = wave.open(file,'rb')  
#instantiate PyAudio  
p = pyaudio.PyAudio()  
#open stream  
stream = p.open(format = p.get_format_from_width(f.getsampwidth()),  
                channels = f.getnchannels(),  
                rate = f.getframerate(),  
                output = True)  

#read data  
data = f.readframes(chunk)  

#play stream  
while data:  
    stream.write(data)  
    data = f.readframes(chunk)

while args.loop:
    #open a wav format music  
    f = wave.open(file,'rb')  
    #instantiate PyAudio  
    p = pyaudio.PyAudio()  
    #open stream  
    stream = p.open(format = p.get_format_from_width(f.getsampwidth()),  
                    channels = f.getnchannels(),  
                    rate = f.getframerate(),  
                    output = True)  
    #read data  
    data = f.readframes(chunk)  

    #play stream  
    while data:  
        stream.write(data)  
        data = f.readframes(chunk)  

#stop stream  
stream.stop_stream()  
stream.close()  

#close PyAudio  
p.terminate() 

exit(0)
