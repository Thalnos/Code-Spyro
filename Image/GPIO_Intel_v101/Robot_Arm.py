#!/usr/bin/env python

#-------------------------------------------------------#
#                                                       #
#              Name: Robot_Arm.py                       #
#         Author: James Clarke, Pridopia.               #
#       Website: http://www.pridopia.co.uk              #
#              Date: 05 / 03 / 14                       #
#                Version: 1.01                          #
#                                                       #
#-------------------------------------------------------#
#
#	Patch Notes 1.01:
#
#	Fixed location for GPIO_Intel to lib folder.
#
# Copyright 2013 Pridopia (www.pridopia.co.uk)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIC,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either-express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import lib.GPIO_Intel as GPIO
import time, sys, os

GPIO = GPIO.Intel()

Motors = {
	"A": ("IO2", "M5"),
	"B": ("IO3", "M5"),
	"C": ("IO4", "M4"),
	"D": ("IO5", "M4"),
	"E": ("IO6", "M3"),
	"F": ("IO7", "M3"),
	"G": ("IO8", "M2"),
	"H": ("IO9", "M2"),
	"I": ("IO10", "M1"),
	"J": ("IO11", "M1")
}

Dnull = open('/dev/null', 'w')

try:
	from msvcrt import kbhit
except:
	import termios, fcntl, sys, os
	def kbhit():
		fd = sys.stdin.fileno()
		oldterm = termios.tcgetattr(fd)
		newattr = termios.tcgetattr(fd)
		newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(fd, termios.TCSANOW, newattr)
		oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
		fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
		try:
			while True:
				try:
					c = sys.stdin.read(1)
					sys.stdin.flush()
					return c
				except IOError:
					return "False"
		finally:
			termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
			fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)



for Key in Motors:
	GPIO.setup(Motors[Key][0])

while True:
	Key = kbhit()
	Key = Key.upper()
	if Key in Motors:
		print " " + Motors[Key][1]
		GPIO.output(Motors[Key][0], 1)
		time.sleep(0.05)
		GPIO.output(Motors[Key][0], 0)
