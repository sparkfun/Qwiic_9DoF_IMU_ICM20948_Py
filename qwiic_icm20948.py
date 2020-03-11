#-----------------------------------------------------------------------------
# qwiic_icm20948.py
#
# Python library for the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic).
#
# https://www.sparkfun.com/products/15335
#
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, March 2020
# 
# This python library supports the SparkFun Electroncis qwiic 
# qwiic sensor/board ecosystem 
#
# More information on qwiic is at https:// www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#==================================================================================
# Copyright (c) 2020 SparkFun Electronics
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

"""
qwiic_icm20948
============
Python module for the [SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)](https://www.sparkfun.com/products/15335)

This python package is a port of the existing [SparkFun ICM-20948 Arduino Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)

This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

"""
#-----------------------------------------------------------------------------

import qwiic_i2c

# Define the device name and I2C addresses. These are set in the class defintion 
# as class variables, making them avilable without having to create a class instance.
# This allows higher level logic to rapidly create a index of qwiic devices at 
# runtine
#
# The name of this device 
_DEFAULT_NAME = "Qwiic ICM20948"

# Some devices have multiple availabel addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the 
# device.
_AVAILABLE_I2C_ADDRESS = [0x69, 0x68]

# define our valid chip IDs
_validChipIDs = [0xEA]

# define the class that encapsulates the device being created. All information associated with this
# device is encapsulated by this class. The device class should be the only value exported 
# from this module.

class QwiicIcm20948(object):
	"""
	QwiicIcm20948

		:param address: The I2C address to use for the device. 
						If not provided, the default address is used.
		:param i2c_driver: An existing i2c driver object. If not provided 
						a driver object is created. 
		:return: The ICM20948 device object.
		:rtype: Object
	"""
	# Constructor
	device_name			= _DEFAULT_NAME
	available_addresses	= _AVAILABLE_I2C_ADDRESS

 # Generalized
	REG_BANK_SEL = 0x7F 

	 # Gyroscope and Accelerometer
	 # User Bank 0
	AGB0_REG_WHO_AM_I = 0x00 
		 # Break
	AGB0_REG_USER_CTRL = 0x03 
		 # Break
	AGB0_REG_LP_CONFIG = 0x05 
	AGB0_REG_PWR_MGMT_1 = 0x06
	AGB0_REG_PWR_MGMT_2 = 0x07
		 # Break
	AGB0_REG_INT_PIN_CONFIG = 0x0F 
	AGB0_REG_INT_ENABLE = 0x10
	AGB0_REG_INT_ENABLE_1 = 0x11
	AGB0_REG_INT_ENABLE_2 = 0x12
	AGB0_REG_INT_ENABLE_3 = 0x13
		 # Break
	AGB0_REG_I2C_MST_STATUS = 0x17 
		 # Break
	AGB0_REG_INT_STATUS = 0x19 
	AGB0_REG_INT_STATUS_1 = 0x1A
	AGB0_REG_INT_STATUS_2 = 0x1B
	AGB0_REG_INT_STATUS_3 = 0x1C
		 # Break
	AGB0_REG_DELAY_TIMEH = 0x28 
	AGB0_REG_DELAY_TIMEL = 0x29
		 # Break
	AGB0_REG_ACCEL_XOUT_H = 0x2D 
	AGB0_REG_ACCEL_XOUT_L = 0x2E
	AGB0_REG_ACCEL_YOUT_H = 0x2F
	AGB0_REG_ACCEL_YOUT_L = 0x30
	AGB0_REG_ACCEL_ZOUT_H = 0x31
	AGB0_REG_ACCEL_ZOUT_L = 0x32
	AGB0_REG_GYRO_XOUT_H = 0x33
	AGB0_REG_GYRO_XOUT_L = 0x34
	AGB0_REG_GYRO_YOUT_H = 0x35
	AGB0_REG_GYRO_YOUT_L = 0x36
	AGB0_REG_GYRO_ZOUT_H = 0x37
	AGB0_REG_GYRO_ZOUT_L = 0x38
	AGB0_REG_TEMP_OUT_H  = 0x39
	AGB0_REG_TEMP_OUT_L = 0x3A
	AGB0_REG_EXT_SLV_SENS_DATA_00 = 0x3B
	AGB0_REG_EXT_SLV_SENS_DATA_01 = 0x3C
	AGB0_REG_EXT_SLV_SENS_DATA_02 = 0x3D
	AGB0_REG_EXT_SLV_SENS_DATA_03 = 0x3E
	AGB0_REG_EXT_SLV_SENS_DATA_04 = 0x3F
	AGB0_REG_EXT_SLV_SENS_DATA_05 = 0x40
	AGB0_REG_EXT_SLV_SENS_DATA_06 = 0x41
	AGB0_REG_EXT_SLV_SENS_DATA_07 = 0x42
	AGB0_REG_EXT_SLV_SENS_DATA_08 = 0x43
	AGB0_REG_EXT_SLV_SENS_DATA_09 = 0x44
	AGB0_REG_EXT_SLV_SENS_DATA_10 = 0x45
	AGB0_REG_EXT_SLV_SENS_DATA_11 = 0x46
	AGB0_REG_EXT_SLV_SENS_DATA_12 = 0x47
	AGB0_REG_EXT_SLV_SENS_DATA_13 = 0x48
	AGB0_REG_EXT_SLV_SENS_DATA_14 = 0x49
	AGB0_REG_EXT_SLV_SENS_DATA_15 = 0x4A
	AGB0_REG_EXT_SLV_SENS_DATA_16 = 0x4B
	AGB0_REG_EXT_SLV_SENS_DATA_17 = 0x4C
	AGB0_REG_EXT_SLV_SENS_DATA_18 = 0x4D
	AGB0_REG_EXT_SLV_SENS_DATA_19 = 0x4E
	AGB0_REG_EXT_SLV_SENS_DATA_20 = 0x4F
	AGB0_REG_EXT_SLV_SENS_DATA_21 = 0x50
	AGB0_REG_EXT_SLV_SENS_DATA_22 = 0x51
	AGB0_REG_EXT_SLV_SENS_DATA_23 = 0x52
		 # Break
	AGB0_REG_FIFO_EN_1 = 0x66 
	AGB0_REG_FIFO_EN_2 = 0x67
	AGB0_REG_FIFO_MODE = 0x69
		 # Break
	AGB0_REG_FIFO_COUNT_H = 0x70 
	AGB0_REG_FIFO_COUNT_L = 0x71
	AGB0_REG_FIFO_R_W = 0x72
		 # Break
	AGB0_REG_DATA_RDY_STATUS = 0x74 
		 # Break
	AGB0_REG_FIFO_CFG = 0x76 
		 # Break
	AGB0_REG_MEM_START_ADDR 	= 0x7C 		 # Hmm  Invensense thought they were sneaky not listing these locations on the datasheet...
	AGB0_REG_MEM_R_W 			= 0x7D 		 # These three locations seem to be able to access some memory within the device
	AGB0_REG_MEM_BANK_SEL 		= 0x7E 		 # And that location is also where the DMP image gets loaded
	AGB0_REG_REG_BANK_SEL = 0x7F 

	 # Bank 1
	AGB1_REG_SELF_TEST_X_GYRO = 0x02 
	AGB1_REG_SELF_TEST_Y_GYRO = 0x03
	AGB1_REG_SELF_TEST_Z_GYRO = 0x04
		 # Break
	AGB1_REG_SELF_TEST_X_ACCEL = 0x0E 
	AGB1_REG_SELF_TEST_Y_ACCEL = 0x0F
	AGB1_REG_SELF_TEST_Z_ACCEL = 0x10
		 # Break
	AGB1_REG_XA_OFFS_H = 0x14 
	AGB1_REG_XA_OFFS_L = 0x15
		 # Break
	AGB1_REG_YA_OFFS_H = 0x17 
	AGB1_REG_YA_OFFS_L = 0x18
		 # Break
	AGB1_REG_ZA_OFFS_H = 0x1A 
	AGB1_REG_ZA_OFFS_L = 0x1B
		 # Break
	AGB1_REG_TIMEBASE_CORRECTION_PLL = 0x28 
		 # Break
	AGB1_REG_REG_BANK_SEL = 0x7F 

	 # Bank 2
	AGB2_REG_GYRO_SMPLRT_DIV = 0x00 
	AGB2_REG_GYRO_CONFIG_1 = 0x01
	AGB2_REG_GYRO_CONFIG_2 = 0x02
	AGB2_REG_XG_OFFS_USRH = 0x03
	AGB2_REG_XG_OFFS_USRL = 0x04
	AGB2_REG_YG_OFFS_USRH = 0x05
	AGB2_REG_YG_OFFS_USRL = 0x06
	AGB2_REG_ZG_OFFS_USRH = 0x07
	AGB2_REG_ZG_OFFS_USRL = 0x08
	AGB2_REG_ODR_ALIGN_EN = 0x09
		 # Break
	AGB2_REG_ACCEL_SMPLRT_DIV_1 = 0x10 
	AGB2_REG_ACCEL_SMPLRT_DIV_2 = 0x11
	AGB2_REG_ACCEL_INTEL_CTRL = 0x12
	AGB2_REG_ACCEL_WOM_THR = 0x13
	AGB2_REG_ACCEL_CONFIG = 0x14
	AGB2_REG_ACCEL_CONFIG_2 = 0x15
		 # Break
	AGB2_REG_FSYNC_CONFIG = 0x52 
	AGB2_REG_TEMP_CONFIG = 0x53
	AGB2_REG_MOD_CTRL_USR = 0x54
		 # Break
	AGB2_REG_REG_BANK_SEL = 0x7F 

	 # Bank 3
	AGB3_REG_I2C_MST_ODR_CONFIG = 0x00 
	AGB3_REG_I2C_MST_CTRL = 0x01
	AGB3_REG_I2C_MST_DELAY_CTRL = 0x02
	AGB3_REG_I2C_SLV0_ADDR = 0x03
	AGB3_REG_I2C_SLV0_REG = 0x04
	AGB3_REG_I2C_SLV0_CTRL = 0x05
	AGB3_REG_I2C_SLV0_DO = 0x06
	AGB3_REG_I2C_SLV1_ADDR = 0x07
	AGB3_REG_I2C_SLV1_REG = 0x08
	AGB3_REG_I2C_SLV1_CTRL = 0x09
	AGB3_REG_I2C_SLV1_DO = 0x0A
	AGB3_REG_I2C_SLV2_ADDR = 0x0B
	AGB3_REG_I2C_SLV2_REG = 0x0C
	AGB3_REG_I2C_SLV2_CTRL = 0x0D
	AGB3_REG_I2C_SLV2_DO = 0x0E
	AGB3_REG_I2C_SLV3_ADDR = 0x0F
	AGB3_REG_I2C_SLV3_REG = 0x10
	AGB3_REG_I2C_SLV3_CTRL = 0x11
	AGB3_REG_I2C_SLV3_DO = 0x12
	AGB3_REG_I2C_SLV4_ADDR = 0x13
	AGB3_REG_I2C_SLV4_REG = 0x14
	AGB3_REG_I2C_SLV4_CTRL = 0x15
	AGB3_REG_I2C_SLV4_DO = 0x16
	AGB3_REG_I2C_SLV4_DI = 0x17
		 # Break
	AGB3_REG_REG_BANK_SEL = 0x7F 
	
	 # Magnetometer
	M_REG_WIA2 = 0x01 
		 # Break
	M_REG_ST1 = 0x10 
	M_REG_HXL = 0x11
	M_REG_HXH = 0x12
	M_REG_HYL = 0x13
	M_REG_HYH = 0x14
	M_REG_HZL = 0x15
	M_REG_HZH = 0x16
	M_REG_ST2 = 0x18
		 # Break
	M_REG_CNTL2 = 0x31 
	M_REG_CNTL3 = 0x32
	M_REG_TS1 = 0x33
	M_REG_TS2 = 0x34

	# Constructor
	def __init__(self, address=None, i2c_driver=None):

		# Did the user specify an I2C address?
		self.address = address if address != None else self.available_addresses[0]

		# load the I2C driver if one isn't provided

		if i2c_driver == None:
			self._i2c = qwiic_i2c.getI2CDriver()
			if self._i2c == None:
				print("Unable to load I2C driver for this platform.")
				return
		else:
			self._i2c = i2c_driver


		# create a dictionary to stash our calibration data for the sensor
		#self.calibration={}

		#self.t_fine=0

		#self._referencePressure = 101325.0

	# ----------------------------------
	# isConnected()
	#
	# Is an actual board connected to our system?

	def isConnected(self):
		""" 
			Determine if a ICM20948 device is conntected to the system..

			:return: True if the device is connected, otherwise False.
			:rtype: bool

		"""
		return qwiic_i2c.isDeviceConnected(self.address)

	# ----------------------------------
	# begin()
	#
	# Initialize the system/validate the board. 
	def begin(self):
		""" 
			Initialize the operation of the ICM20948 module

			:return: Returns true of the initializtion was successful, otherwise False.
			:rtype: bool

		"""
		# are we who we need to be?
		chipID = self._i2c.readByte(self.address, self.AGB0_REG_WHO_AM_I)
		if not chipID in _validChipIDs:
			print("Invalid Chip ID: 0x%.2X" % chipID)
			return False
		
		return True

	#----------------------------------------------------------------
	# Mode of the sensor 

	def setMode(self, mode):
		""" 
			Set the operational mode of the sensor.

			:param mode: One of the class constant values.
					MODE_SLEEP, MODE_FORCED, MODE_NORMAL 

			:return: No return value

		"""
		if mode > 0b11:
			mode = 0  # Error check. Default to sleep mode
	
		controlData = self._i2c.readByte(self.address, self.BME280_CTRL_MEAS_REG)
		controlData &= (~( (1<<1) | (1<<0) ) ) & 0xFF # Clear the mode[1:0] bits - note we just want a byte
		controlData |= mode   # Set
		self._i2c.writeByte(self.address, self.BME280_CTRL_MEAS_REG, controlData)


	def getMode(self):
		""" 
			Returns the operational mode of the sensor.

			:return: The current operational mode
			:rtype: MODE_SLEEP, MODE_FORCED, or MODE_NORMAL

		"""
		controlData = self._i2c.readByte(self.address, self.BME280_CTRL_MEAS_REG)
		return controlData & 0b00000011

	# Make the mode a property of this object
	mode = property(getMode, setMode)

	#----------------------------------------------------------------
	# Set the standby bits in the config register
	# tStandby can be:
	#   0, 0.5ms
	#   1, 62.5ms
	#   2, 125ms
	#   3, 250ms
	#   4, 500ms
	#   5, 1000ms
	#   6, 10ms
	#   7, 20ms
	def setStandbyTime(self, timeSetting):
		"""
		Set the standby bits in the BME280s config register

		:param timeSetting: The standby time bits - acceptable values
		 				0 = 0.5ms
	  					1 = 62.5ms
	  					2 = 125ms
	  					3 = 250ms
	  					4 = 500ms
	  					5 = 1000ms
	  					6 = 10ms
	  					7 = 20ms

		:return: No return value 

		"""

		if timeSetting > 0b111 :
			timeSetting = 0 # Error check. Default to 0.5ms
		
		controlData = self._i2c.readByte(self.address, self.BME280_CONFIG_REG)
		controlData &= ( ~( (1<<7) | (1<<6) | (1<<5) )) & 0xff # Clear the 7/6/5 bits
		controlData |= (timeSetting << 5) # Align with bits 7/6/5
		self._i2c.writeByte(self.address, self.BME280_CONFIG_REG, controlData)

	# Make standby time a property
	standby_time = property()
	standby_time = standby_time.setter(setStandbyTime)

	#---------------------------------------------------------------- 
	# Set the filter bits in the config register
	# filter can be off or number of FIR coefficients to use:
	#   0, filter off
	#   1, coefficients = 2
	#   2, coefficients = 4
	#   3, coefficients = 8
	#   4, coefficients = 16
	def setFilter(self, filterSetting):
		"""
		Set the filter bits in the BME280s config register

		:param filterSetting: The filter bits for the BME280. Acceptable values
						0 = filter off
	  					1 = coefficients = 2
	  					2 = coefficients = 4
	  					3 = coefficients = 8
	  					4 = coefficients = 16

		:return: No return value 

		"""
		if filterSetting > 0b111 : 
			filterSetting = 0 # Error check. Default to filter off
		
		controlData = self._i2c.readByte(self.address, self.BME280_CONFIG_REG)
		controlData &= (~( (1<<4) | (1<<3) | (1<<2) ) ) & 0xFF # Clear the 4/3/2 bits
		controlData |= (filterSetting << 2) # Align with bits 4/3/2
		self._i2c.writeByte(self.address, self.BME280_CONFIG_REG, controlData)

	filter = property()
	filter = filter.setter(setFilter)

	#----------------------------------------------------------------	
	# Set the temperature oversample value
	# 0 turns off temp sensing
	# 1 to 16 are valid over sampling values
	def setTempOverSample(self, overSampleAmount):
		"""
		Set the temperature oversample value

		:param overSampleAmount: The temperature oversample value. Acceptable values
				0 = turns off temp sensing
				1 to 16 are valid over sampling values

		:return: No return value 
		"""
		overSampleAmount = self.checkSampleValue(overSampleAmount) # Error check
		
		originalMode = self.getMode() # Get the current mode so we can go back to it at the end
		
		self.setMode(self.MODE_SLEEP) # Config will only be writeable in sleep mode, so first go to sleep mode
	
		# Set the osrs_t bits (7, 6, 5) to overSampleAmount
		controlData = self._i2c.readByte(self.address, self.BME280_CTRL_MEAS_REG)
		controlData &= (~( (1<<7) | (1<<6) | (1<<5) )) & 0xFF # Clear bits 765
		controlData |= overSampleAmount << 5 # Align overSampleAmount to bits 7/6/5
		self._i2c.writeByte(self.address, self.BME280_CTRL_MEAS_REG, controlData)
		
		self.setMode(originalMode) # Return to the original user's choice

	tempature_oversample = property()
	tempature_oversample = tempature_oversample.setter(setTempOverSample)

	# Set the pressure oversample value
	# 0 turns off pressure sensing
	# 1 to 16 are valid over sampling values
	def setPressureOverSample(self, overSampleAmount):
		"""
		Set the pressure oversample value

		:param overSampleAmount: The pressure oversample value. Acceptable values
				0 = turns off pressure sensing
				1 to 16 are valid over sampling values

		:return: No return value 
		"""
		overSampleAmount = self.checkSampleValue(overSampleAmount) # Error check
		
		originalMode = self.getMode() # Get the current mode so we can go back to it at the end
		
		self.setMode(self.MODE_SLEEP) # Config will only be writeable in sleep mode, so first go to sleep mode
	
		# Set the osrs_p bits (4, 3, 2) to overSampleAmount
		controlData = self._i2c.readByte(self.address, self.BME280_CTRL_MEAS_REG)
		controlData &= (~( (1<<4) | (1<<3) | (1<<2) )) & 0xFF  # Clear bits 432
		controlData |= overSampleAmount << 2 # Align overSampleAmount to bits 4/3/2
		self._i2c.writeByte(self.address, self.BME280_CTRL_MEAS_REG, controlData)
		
		self.setMode(originalMode) # Return to the original user's choice

	pressure_oversample = property()
	pressure_oversample = pressure_oversample.setter(setPressureOverSample)

	#----------------------------------------------------------------	
	# Set the humidity oversample value
	# 0 turns off humidity sensing
	# 1 to 16 are valid over sampling values
	def setHumidityOverSample(self, overSampleAmount):
		"""
		Set the humidity oversample value

		:param overSampleAmount: The humidity oversample value. Acceptable values
				0 = turns off humidity sensing
				1 to 16 are valid over sampling values

		:return: No return value 
		"""
		overSampleAmount = self.checkSampleValue(overSampleAmount) # Error check
		
		originalMode = self.getMode() # Get the current mode so we can go back to it at the end
		
		self.setMode(self.MODE_SLEEP) # Config will only be writeable in sleep mode, so first go to sleep mode
	
		# Set the osrs_h bits (2, 1, 0) to overSampleAmount
		controlData = self._i2c.readByte(self.address, self.BME280_CTRL_HUMIDITY_REG)
		controlData &= (~( (1<<2) | (1<<1) | (1<<0) )) & 0xFF # Clear bits 2/1/0
		controlData |= overSampleAmount << 0 # Align overSampleAmount to bits 2/1/0
		self._i2c.writeByte(self.address, self.BME280_CTRL_HUMIDITY_REG, controlData)
	
		self.setMode(originalMode) # Return to the original user's choice

	humidity_oversample = property()
	humidity_oversample = humidity_oversample.setter(setHumidityOverSample)

	#----------------------------------------------------------------	
	# Validates an over sample value
	# Allowed values are 0 to 16
	# These are used in the humidty, pressure, and temp oversample functions
	def checkSampleValue(self, userValue):
		"""
		Validates an over sample value

		:param userValue: The oversample value to check. 
				Allowed values are 0 to 16
				These are used in the humidty, pressure, and temp oversample functions

		:return: Valid oversample value
		:rtype: int
		"""
		_valueMap = { 0: 0, 1: 1, 2: 2, 4: 3, 8: 4, 16: 5}

		return _valueMap[userValue] if userValue in _valueMap.keys() else 1
		
	# Check the measuring bit and return true while device is taking measurement
	def isMeasuring(self):
		"""
		Return if the sensor is measuring or not

		:return: True if the sensor is measuring, else False
		:rvalue: boolean
		"""

		stat = self._i2c.readByte(self.address, self.BME280_STAT_REG)
		return  True if stat & (1<<3) else False # If the measuring bit (3) is set, return true

	
	# Strictly resets.  Run .begin() afterwards
	def reset( self ):
		"""
		Resets the sensor. If called, the begin method must be called before 
		using the sensor.

		"""
		self._i2c.writeByte(self.address, self.BME280_RST_REG, 0xB6)

	# ****************************************************************************# 
	# 
	#   Pressure Section
	# 
	# ****************************************************************************# 
	def readFloatPressure( self ):
		"""
		Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
		Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa

		:return: Pressure in Pa
		:rtype: integer

		"""
		#  Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
		#  Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa

		buffer = self._i2c.readBlock(self.address, self.BME280_PRESSURE_MSB_REG, 3)
		adc_P = (buffer[0] << 12) | (buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F)
		
		var1 = self.t_fine - 128000
		var2 = var1 * var1 * self.calibration["dig_P6"]
		var2 = var2 + ((var1 * self.calibration["dig_P5"])<<17)
		var2 = var2 + (self.calibration["dig_P4"] <<35)
		var1 = ((var1 * var1 * self.calibration["dig_P3"])>>8) + ((var1 * self.calibration["dig_P2"])<<12)
		var1 = ( (1 << 47) + var1 )*(self.calibration["dig_P1"])>>33

		if var1 == 0:
			return 0  #  avoid exception caused by division by zero

		p_acc = 1048576 - adc_P
		p_acc = (((p_acc<<31) - var2)*3125)//var1

		var1 = ((self.calibration["dig_P9"]) * (p_acc>>13) * (p_acc>>13)) >> 25
		var2 = ((self.calibration["dig_P8"]) * p_acc) >> 19
		p_acc = ((p_acc + var1 + var2) >> 8) + ((self.calibration["dig_P7"])<<4)
		
		return p_acc / 256.0
		
	pressure = property(readFloatPressure)

	#----------------------------------------------------------------	
	# Sets the internal variable _referencePressure so the 
	def setReferencePressure(self, refPressure):
		"""
		Sets the referance pressure for the sensor measurments.

		:param refPressure: The referance pressure to use.

		:return: No return value 
		"""
		self._referencePressure = float(refPressure)
	
	# Return the local reference pressure
	def getReferencePressure(self):
		"""
		Get the current reference pressure for the sensor.

		:return: The current reference pressure.
		:rtype: float
		"""
		return self._referencePressure
	
	reference_pressure = property(getReferencePressure, setReferencePressure)

	#----------------------------------------------------------------	
	def readFloatAltitudeMeters( self ):
		"""
		Return the current Altitude in meters

		:return: The current altitude in meters
		:rtype: float
		"""
		# heightOutput = ((float)-45846.2)*(pow(((float)readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1);

		return (-44330.77)*(math.pow((self.readFloatPressure()/self._referencePressure), 0.190263) - 1.0) # Corrected, see issue 30
	
	altitude_meters = property(readFloatAltitudeMeters)

	#----------------------------------------------------------------
	def readFloatAltitudeFeet( self ):
		"""
		Return the current Altitude in feet

		:return: The current altitude in feets
		:rtype: float
		"""
		return self.readFloatAltitudeMeters() * 3.28084
	
	altitude_feet = property(readFloatAltitudeFeet)


	# ****************************************************************************# 
	# 
	#   Humidity Section
	# 
	# ****************************************************************************# 
	def readFloatHumidity( self ):
		"""
		Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
		Output value of "47445" represents 47445/1024 = 46. 33 %RH

		:return: The current humidity value
		:rtype: float
		"""
		#  Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
		#  Output value of "47445" represents 47445/1024 = 46. 33 %RH

		buffer = self._i2c.readBlock(self.address, self.BME280_HUMIDITY_MSB_REG, 2)
		adc_H = (buffer[0] << 8) | buffer[1]
		
		var1 = (self.t_fine - 76800)
		var1 = (((((adc_H << 14) - ((self.calibration["dig_H4"]) << 20) - ((self.calibration["dig_H5"]) * var1)) + \
			(16384)) >> 15) * (((((((var1 * (self.calibration["dig_H6"])) >> 10) * (((var1 * (self.calibration["dig_H3"])) >> 11) + (32768))) >> 10) + (2097152)) * \
			(self.calibration["dig_H2"]) + 8192) >> 14))
		var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (self.calibration["dig_H1"])) >> 4))
		var1 = 0 if var1 < 0 else  var1 
		var1 = 419430400 if var1 > 419430400 else var1

	
		return (var1>>12) / 1024.0

	humidity = property(readFloatHumidity)

	# ****************************************************************************# 
	# 
	#   Temperature Section
	# 
	# ****************************************************************************# 
	
	def readTempC( self ):
		"""
		Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
		 t_fine carries fine temperature as global value

		:return: The current temperature in C.
		:rtype: float
		"""
		#  Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
		#  t_fine carries fine temperature as global value
	
		# get the reading (adc_T);

		buffer = self._i2c.readBlock(self.address, self.BME280_TEMPERATURE_MSB_REG, 3)
		adc_T = (buffer[0] << 12) | (buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F)
	
		# By datasheet, calibrate
	
		var1 = ((((adc_T>>3) - (self.calibration["dig_T1"]<<1))) * (self.calibration["dig_T2"])) >> 11
		var2 = (((((adc_T>>4) - (self.calibration["dig_T1"])) * ((adc_T>>4) - (self.calibration["dig_T1"]))) >> 12) * \
				(self.calibration["dig_T3"])) >> 14
		self.t_fine = var1 + var2
		output = (self.t_fine * 5 + 128) >> 8
	
		return output / 100 + _settings["tempCorrection"]
		
	temperature_celsius = property(readTempC)

	#----------------------------------------------------------------	
	def readTempF( self ):
		"""
		Returns temperature in Deg F, resolution is 0.01 DegF. Output value of "5123" equals 51.23 DegF.
		 t_fine carries fine temperature as global value

		:return: The current temperature in F.
		:rtype: float
		"""
		output = self.readTempC()
		return (output * 9) / 5 + 32

	temperature_fahrenheit = property(readTempF)

	# ****************************************************************************# 
	# 
	#   Dew point Section
	# 
	# ****************************************************************************# 
	#  Returns Dew point in DegC

	def dewPointC(self):
		"""
		Returns the Dew point in degrees C. 

		:return: The current dewpoint in C.
		:rtype: float
		"""
		celsius = self.readTempC() 
		humidity = self.readFloatHumidity()
		#  (1) Saturation Vapor Pressure = ESGG(T)

		RATIO = 373.15 / (273.15 + celsius)
		RHS = -7.90298 * (RATIO - 1)
		RHS += 5.02808 * math.log10(RATIO)
		RHS += -1.3816e-7 * (math.pow(10, (11.344 * (1 - 1/RATIO ))) - 1) 
		RHS += 8.1328e-3 * (math.pow(10, (-3.49149 * (RATIO - 1))) - 1)
		RHS += math.log10(1013.246)
		       #  factor -3 is to adjust units - Vapor Pressure SVP * humidity
		VP = math.pow(10, RHS - 3) * humidity
		       #  (2) DEWPOINT = F(Vapor Pressure)
		T = math.log(VP/0.61078)   #  temp var
		return (241.88 * T) / (17.558 - T)
	
	dewpoint_celsius = property(dewPointC)

	#----------------------------------------------------------------	
	#  Returns Dew point in DegF
	def dewPointF(self):
		"""
		Returns the Dew point in degrees F. 

		:return: The current dewpoint in F.
		:rtype: float
		"""
		return self.dewPointC() * 1.8 + 32 # Convert C to F

	dewpoint_fahrenheit = property(dewPointF)