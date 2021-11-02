#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import os
import sys
import argparse

env_path = os.path.join(os.path.dirname(__file__), '.')
if env_path not in sys.path:
    sys.path.append(env_path)

import gStreamer
from config import configs

def main():
    parser = argparse.ArgumentParser(description='Run ROS-GSTreamer interface.')

    parser.add_argument('-c', '--camera', type=str, default='laptop', help='Camera Type. Default: Laptop camera.')
    parser.add_argument('-v', '--version',  help='show program version", action="store_true')
    parser.add_argument("-e", "--endpoint", type=str,   default='newpreprod', help='Endpoint')
    args, unknown = parser.parse_known_args()
    if args.version:
	    print("Version 0.8.")
	    exit()

    gStremerObj = gStreamer.gStreamer(args.endpoint, args.camera)
    gStremerObj.run()


if __name__ == '__main__':
    main()
