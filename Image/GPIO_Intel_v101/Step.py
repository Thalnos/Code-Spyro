#!/usr/bin/env python

#-------------------------------------------------------#
#                                                       #
#              Name: Step.py                            #
#         Author: James Clarke, Pridopia.               #
#       Website: http://www.pridopia.co.uk              #
#              Date: 05 / 03 / 14                       #
#                Version: 1.01                          #
#                                                       #
#-------------------------------------------------------#
#
#	Patch Notes 1.01:
#
#	Fixed GPIO_Intel Location to lib folder.
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

StepA = ["IO2", "IO3", "IO4", "IO5"]
StepB = ["IO6", "IO7", "IO8", "IO9"]

GPIO.stepper(StepA, 500, "CW")

GPIO.stepper(StepB, 500, "CC")

GPIO.cleanup()
exit(0)
