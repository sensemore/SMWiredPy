import SMWiredPy

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
sampling_frequency = 6400
sample_size = 200


measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

#Measurement results are native python lists
result_acc_x = measurement_result[0]
result_acc_y = measurement_result[1]
result_acc_z = measurement_result[2]

print("Acc-X:",result_acc_x)
print("Acc-Y:",result_acc_y)
print("Acc-Z:",result_acc_z)

"""
Also there are telemetries calculated in wired, we can also take it by calling get_all_telemetry
"""

telemetries = wired_network.get_all_telemetry(mac)
print(telemetries)


"""
	With numpy and matplotlib, plot the accelerometer raw data
	# import numpy as np
	# from matplotlib import pyplot as plt

	# plt.plot(result_acc_x)
	# plt.show()
"""