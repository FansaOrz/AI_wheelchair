#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import serial

port = serial.Serial('/dev/ttyUSB0', 9600)
line = port.readline(150)
while line:
    print line
    print "================"
    try:
        line = port.readline(150)
    except RuntimeError:
        print ""
