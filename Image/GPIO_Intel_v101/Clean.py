#!/usr/bin/env python

#-------------------------------------------------------#
#                                                       #
#              Name: GPIO_Intel.py                      #
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

GPIO = GPIO.Intel()

IO = [
32,18,28,17,24,27,
26,19,16,25,38,39
]

Ana = [
44, 45, 46, 47, 48, 49
]

PWM = [
1, 3, 4, 5, 6, 7
]

def Write(value, file):
	with open(file, 'w') as File:
		File.write(str(value))
	return

for Pin in IO:
	Write('0', '/sys/class/gpio/gpio{}/value'.format(Pin))

for Pin in PWM:
	Write('0', '/sys/class/pwm/pwmchip0/pwm{}/enable'.format(Pin))

print "Cleared all I/O and PWM."
