#!/usr/bin/env python3

import spidev
import time
import argparse 
import sys
import navio2.mpu9250
import navio2.util

navio2.util.check_apm()

imu_type = 'mpu'

if imu_type == 'mpu':
    print("Selected: MPU9250")
    imu = navio2.mpu9250.MPU9250()
elif imu_type == 'lsm':
    print("Selected: LSM9DS1")
    imu = navio2.lsm9ds1.LSM9DS1()
else:
    print("Wrong sensor name. Select: mpu or lsm")
    sys.exit(1)

if imu.testConnection():
    print("Connection established: True")
else:
    sys.exit("Connection established: False")

imu.initialize()

time.sleep(1)

while True:
	# imu.read_all()
	# imu.read_gyro()
	# imu.read_acc()
	# imu.read_temp()
	# imu.read_mag()

	# print "Accelerometer: ", imu.accelerometer_data
	# print "Gyroscope:     ", imu.gyroscope_data
	# print "Temperature:   ", imu.temperature
	# print "Magnetometer:  ", imu.magnetometer_data

	# time.sleep(0.1)

	m9a, m9g, m9m = imu.getMotion9()

	print("Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]))
	print(" Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]))
	print(" Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2]))

	time.sleep(0.5)