#!/usr/bin/env python
#-----------------------------------------------------------------------------
# ex1_qwiic_ICM20948.py
#
# Simple Example for the Qwiic ICM20948 Device
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, March 2020
# 
# This python library supports the SparkFun Electroncis qwiic 
# qwiic sensor/board ecosystem on a Raspberry Pi (and compatable) single
# board computers. 
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal 
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
# SOFTWARE.
#==================================================================================
# Example 1
#

from __future__ import print_function
import qwiic_icm20948
import time
import sys

#def printRawAGMT():
	# print("RAW. Acc [ ");
	# printPaddedInt16b( agmt.acc.axes.x );
	# print(", ");
	# printPaddedInt16b( agmt.acc.axes.y );
	# print(", ");
	# printPaddedInt16b( agmt.acc.axes.z );
	# print(" ], Gyr [ ");
	# printPaddedInt16b( agmt.gyr.axes.x );
	# print(", ");
	# printPaddedInt16b( agmt.gyr.axes.y );
	# print(", ");
	# printPaddedInt16b( agmt.gyr.axes.z );
	# print(" ], Mag [ ");
	# printPaddedInt16b( agmt.mag.axes.x );
	# print(", ");
	# printPaddedInt16b( agmt.mag.axes.y );
	# print(", ");
	# printPaddedInt16b( agmt.mag.axes.z );
	# print(" ], Tmp [ ");
	# printPaddedInt16b( agmt.tmp.val );
	# print(" ]");
	# println();


def runExample():

	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()

	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	IMU.begin()

	while True:
		if IMU.dataReady():
			IMU.getAgmt() # read all axis from sensor, note this also updates all instance variables
			print(IMU.axRaw, IMU.ayRaw, IMU.azRaw, IMU.gxRaw, IMU.gyRaw, IMU.gzRaw, IMU.mxRaw, IMU.myRaw, IMU.mzRaw, IMU.tmpRaw, IMU.magStat1, IMU.magStat2)
    		#printRawAGMT( IMU.agmt );     # Uncomment this to see the raw values, taken directly from the agmt structure
    		#printScaledAGMT( IMU.agmt);   # This function takes into account the sclae settings from when the measurement was made to calculate the values with units
			time.sleep(0.03)
		else:
			print("Waiting for data")
			time.sleep(0.5)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)


