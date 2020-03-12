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
import time

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

	connected = property(isConnected)

	# ----------------------------------
	# setBank()
	#
	# Sets the bank register of the ICM20948 module
	def setBank(self, bank):
		""" 
			Sets the bank register of the ICM20948 module

			:return: Returns true if the bank was a valid value and it was set, otherwise False.
			:rtype: bool

		"""
		if bank > 3:	# Only 4 possible banks
			print("Invalid Bank value: %d" % bank)
			return False			   
		bank = ((bank << 4) & 0x30) # bits 5:4 of REG_BANK_SEL
		#return ICM_20948_execute_w(pdev, REG_BANK_SEL, &bank, 1)
		return self._i2c.writeByte(self.address, self.REG_BANK_SEL, bank)

	# ----------------------------------
	# swReset()
	#
	# Performs a software reset on the ICM20948 module
	def swReset(self):
		""" 
			Performs a software reset on the ICM20948 module

			:return: Returns true if the software reset was successful, otherwise False.
			:rtype: bool

		"""
		# Read the Power Management Register, store in local variable "register"
		self.setBank(0)
		register = self._i2c.readByte(self.address, self.AGB0_REG_PWR_MGMT_1)

		# Set the device reset bit [7]
		register |= (1<<7)

		# Write register
		self.setBank(0)
		return self._i2c.writeByte(self.address, self.AGB0_REG_PWR_MGMT_1, register)		

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
		self.setBank(0)
		chipID = self._i2c.readByte(self.address, self.AGB0_REG_WHO_AM_I)
		if not chipID in _validChipIDs:
			print("Invalid Chip ID: 0x%.2X" % chipID)
			return False
		
		# software reset
		self.swReset()
		time.sleep(.05)

		return True
	

	# def startupDefault(self)
	# 	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	# 	retval = sleep(false);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}

	# 	retval = lowPower(false);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}

	# 	retval = setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	} // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

	# 	ICM_20948_fss_t FSS;
	# 	FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
	# 	FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
	# 	retval = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}

	# 	ICM_20948_dlpcfg_t dlpcfg;
	# 	dlpcfg.a = acc_d473bw_n499bw;
	# 	dlpcfg.g = gyr_d361bw4_n376bw5;
	# 	retval = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}

	# 	retval = enableDLPF(ICM_20948_Internal_Acc, false);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}
	# 	retval = enableDLPF(ICM_20948_Internal_Gyr, false);
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}
	# 	retval = startupMagnetometer();
	# 	if (retval != ICM_20948_Stat_Ok)
	# 	{
	# 		status = retval;
	# 		return status;
	# 	}

	# 	return status;
	# }