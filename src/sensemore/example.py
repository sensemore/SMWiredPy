import SMWiredPy
from matplotlib import pyplot as plt
import numpy as np


wired_network = SMWiredPy.SMWired(port = "/dev/ttyUSB0", configure_network='auto', max_device_number=2)
#Dump the list of found available devices
print("Found available devices:",wired_network.get_available_devices())


devices = wired_network.get_available_devices()

#Print the version of the devices
for device in devices:
	print("Version of '%s' is %s"%(device,wired_network.get_version(device)))


#Take measurement from a specific device in the network

mac = 'CA:B8:31:00:00:55'
accelerometer_range = "16G"
sampling_frequency = 12800
sample_size = 200000

def meas_deneme(mac):
	measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

	result_acc_x = measurement_result[0]
	result_acc_y = measurement_result[1]
	result_acc_z = measurement_result[2]
	plt.plot(result_acc_x)
	plt.show()

def sync_Deneme():
	print("Calling measure sync")
	measurement_map = wired_network.measure_sync(accelerometer_range,sampling_frequency,sample_size)
	
	# measurement_map
	for m in measurement_map.keys():
	#m = "CA:B8:31:00:00:55"
		print("np! m",m,len(measurement_map[m]),len(measurement_map[m][0]))
		result = measurement_map[m]
		#x = np.array(result[0])
		#print(x.shape)
		#x = x - np.mean(x)
		#print(mac,x)
		#plt.plot(x,'.')
		#continue
	
	#plt.legend(list(measurement_map.keys()))


	# import json

	# with open("anil_sync_measurement.json","w+") as f:
	# 	json.dump(measurement_map,f)

	#plt.show()


x = sync_Deneme()
# for mac in wired_network.get_available_devices():
# 	meas_deneme(mac)

#if(wired_network.get_version(mac) != "1.254.14"):
#wired_network.firmware_update(mac,'/home/fbgencer/wiredpy/Wiredv1_251_14.bin')

#exit()
# measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

# result_acc_x = measurement_result[0]
# result_acc_y = measurement_result[1]
# result_acc_z = measurement_result[2]

"""
Also there are telemetries calculated in wired, we can also take it by calling get_all_telemetry
"""

#telemetries = wired_network.get_all_telemetry(mac)
#print(telemetries)


#With numpy and matplotlib, plot the accelerometer raw data
# import numpy as np
# from matplotlib import pyplot as plt

# plt.plot(result_acc_x)
# plt.show()
