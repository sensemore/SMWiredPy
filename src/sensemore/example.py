import SMWiredPy

mac = "7C:DF:A1:C2:29:0C"
network = SMWiredPy.SMWired(port="/dev/ttyUSB1", configure_network=[mac])
self = network
import time
#time.sleep(1)
for i in range(100):
	for device in self.get_available_devices():
		print("Version of '%s' is %s"%(device,self.get_version(device)))
exit()
print(network.get_available_devices())
if(isinstance(mac, str)):
	mac_as_byte = [int(x, 16) for x in mac.split(':')]
# receiver_id = self.device_map[mac].user_defined_id
receiver_id = 0
print("here",receiver_id,mac)
enter_message = [*mac_as_byte]
write_ret = self.write(receiver_id, SMWiredPy.SMCOM_WIRED_MESSAGES.ENTER_FIRMWARE_UPDATER_MODE.value, enter_message, len(enter_message))

exit()
wired_network = SMWiredPy.SMWired(port = "/dev/ttyUSB1", configure_network='auto', max_device_number=2)
#Dump the list of found available devices
print("Found available devices:",wired_network.get_available_devices())

devices = wired_network.get_available_devices()

#Print the version of the devices
for device in devices:
	print("Version of '%s' is %s"%(device,wired_network.get_version(device)))


#Take measurement from a specific device in the network

mac = wired_network.get_available_devices()[0]
accelerometer_range = "16G"
sampling_frequency = 6400
sample_size = 20000


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