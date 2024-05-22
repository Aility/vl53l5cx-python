#!/usr/bin/env python3

import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE
import numpy


print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX()
print("Done!")
vl53.set_resolution(8 * 8)


vl53.set_ranging_frequency_hz(15)
vl53.set_integration_time_ms(20)

vl53.start_ranging()

# before calibration
for i in range(10):
    if vl53.data_ready():
        data = vl53.get_data()
        status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))), (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
        reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8))).astype('float64')
        distance = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8))).astype('float64')
        print(status, reflectance, distance)

vl53.stop_ranging()

# doing calibration
reflectance_percent = 10 # 3% would be the recommended, our target has more reflectance
nb_samples = 16 # we set that to max
distance_mm = 1000 # suppose the target is at 1 meter
vl53.calibrate_xtalk(reflectance_percent, nb_samples, distance_mm)
print("xtalk calc data = ", vl53.get_caldata_xtalk())
print("xtalk margin = ", vl53.get_xtalk_margin())

# After calibration
vl53.start_ranging()
for i in range(10):
    if vl53.data_ready():
        data = vl53.get_data()
        status = numpy.isin(numpy.flipud(numpy.array(data.target_status).reshape((8, 8))), (STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE))
        reflectance = numpy.flipud(numpy.array(data.reflectance).reshape((8, 8))).astype('float64')
        distance = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8))).astype('float64')
        print(status, reflectance, distance)

vl53.stop_ranging()
