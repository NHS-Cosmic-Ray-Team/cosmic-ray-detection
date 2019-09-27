#!/usr/bin/python
# get lines of text from serial port, save them to a file

from __future__ import print_function
import serial, io
import os
import time

# class to create file to save data to
class RotatingFileOpener():
    def __init__(self, path, mode = 'a', prepend = "", append = ""):
        if not os.path.isdir(path):
            raise FileNotFoundError("Can't open directory '{}'.".format(path))
        self._path = path
        self._prepend = prepend
        self._append = append
        self._mode = mode
        self._day = time.localtime().tm_mday
       
    def __enter__(self):
        self._filename = self._format_filename()
        self._file = open(self._filename, self._mode)
        return self
    def __exit__(self, *args):
        return getattr(self._file, '__exit__')(*args)
    def _day_changed(self):
        return self._day != time.localtime().tm_mday
        
    def _format_filename(self):
        return os.path.join(self._path, "{}{}{}".format(self._prepend, time.strftime("NPS-%Y-%m-%d"), self._append))
    def write(self, *args):
        if self._day_changed():
            self._file.close()
            self._file = open(self._format_filename(), self._mode)
        return getattr(self._file, 'write')(*args)
    def __getattr__(self, attr):
        return getattr(self._file, attr)
    def __iter__(self):
        return iter(self._file)

addr  = '/dev/ttyACM0'  # serial port to read data from
baud  = 9600            # baud rate for serial port
    
with serial.Serial(addr,baud) as pt, RotatingFileOpener('/home/pi/Backup',
                                                        prepend = '', append = '.txt') as logger:
    spb = io.TextIOWrapper(io.BufferedRWPair(pt,pt,1),
    encoding = 'ascii', errors ='ignore', newline = '\r', line_buffering = True)
    
    while True:                         
        log = spb.readline()    # read one line of text from serial port
        logger.write(log)       # write line of text to file
        logger.flush()          # make sure it actually gets written out
      
                      
