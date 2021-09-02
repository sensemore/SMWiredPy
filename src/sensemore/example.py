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
sample_size = 100000


def DCOffset(signal):
    signal_np = np.array(signal)
    mean = signal_np.mean()
    return np.subtract(signal_np, mean)

def GRMS(signal):
    signal_np = DCOffset(signal)
    result = np.sqrt((((signal_np)**2)/len(signal)).sum())
    return float(result)

def KURTOSIS(signal):
    signal_np = DCOffset(signal)
    sum_of_4th_power = np.power(signal_np, 4).sum() / len(signal)
    sum_of_squares = np.power((np.power(signal_np, 2).sum()/len(signal)), 2)
    result = sum_of_4th_power / sum_of_squares
    return float(result)

def CLEARANCE(signal):
    signal_np = DCOffset(signal)
    signal_abs = np.abs(signal_np)
    peek = signal_abs.max()
    result = peek/(np.sqrt(signal_abs).sum() / (len(signal)))**2
    return float(result)

def SKEWNESS(signal):
    signal_np = DCOffset(signal)
    sum_of_cubes = np.power(signal_np, 3).sum() / len(signal)
    sum_of_squares = np.power((np.power(signal_np, 2).sum()/len(signal)), 1.5)
    result = sum_of_cubes / sum_of_squares
    return float(result)

def CREST(signal):
    signal_np = DCOffset(signal)
    signal_abs = np.abs(signal_np)
    peek = signal_abs.max()
    result = peek/GRMS(signal)
    return float(result)

def PEAK_TO_PEAK(signal):
	signal_np = DCOffset(signal)
	maxval = np.abs(np.max(signal_np))
	minval = np.abs(np.min(signal_np))
	return maxval + minval


def meas_deneme(mac):
	measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

	result_acc_x = measurement_result[0]
	result_acc_y = measurement_result[1]
	result_acc_z = measurement_result[2]
	plt.plot(result_acc_x)
	plt.show()

def sync_Deneme():
	measurement_map = wired_network.measure_sync(accelerometer_range,sampling_frequency,sample_size)
	
	# measurement_map
	for m in measurement_map.keys():
	#m = "CA:B8:31:00:00:55"
		print("np! m",m,len(measurement_map[m]),len(measurement_map[m][0]))
		import time
		start_time = time.time()
		telem = wired_network.get_all_telemetry(m)
		print("--- %s seconds ---" % (time.time() - start_time))
		
		
		#telem = wired_network.get_all_telemetry(m)
		print(telem)
		result = measurement_map[m]
		x = np.array(result[0])
		y = np.array(result[1])
		z = np.array(result[2])
		

		p2p_check = (np.abs(PEAK_TO_PEAK(result[0])-telem["peak_to_peak"][0])<1e-9) and (np.abs(PEAK_TO_PEAK(result[1])-telem["peak_to_peak"][1])<1e-9) and (np.abs(PEAK_TO_PEAK(result[2])-telem["peak_to_peak"][2])<1e-9)
		crest_check = (np.abs(CREST(result[0])-telem["crest"][0])<1e-9) and (np.abs(CREST(result[1])-telem["crest"][1])<1e-9) and (np.abs(CREST(result[2])-telem["crest"][2])<1e-9)
		clearance_check = (np.abs(CLEARANCE(result[0])-telem["clearance"][0])<1e-9) and (np.abs(CLEARANCE(result[1])-telem["clearance"][1])<1e-9) and (np.abs(CLEARANCE(result[2])-telem["clearance"][2])<1e-9)
		sum_check = (np.abs(np.sum(result[0])-telem["sum"][0])<1e-9) and (np.abs(np.sum(result[1])-telem["sum"][1])<1e-9) and (np.abs(np.sum(result[2])-telem["sum"][2])<1e-9)
		kurtosis_check = (np.abs(KURTOSIS(result[0])-telem["kurtosis"][0])<1e-9) and (np.abs(KURTOSIS(result[1])-telem["kurtosis"][1])<1e-9) and (np.abs(KURTOSIS(result[2])-telem["kurtosis"][2])<1e-9)
		skewness_check = (np.abs(SKEWNESS(result[0])-telem["skewness"][0])<1e-9) and (np.abs(SKEWNESS(result[1])-telem["skewness"][1])<1e-9) and (np.abs(SKEWNESS(result[2])-telem["skewness"][2])<1e-9)
		grms_check = (np.abs(GRMS(result[0])-telem["grms"][0])<1e-9) and (np.abs(GRMS(result[1])-telem["grms"][1])<1e-9) and (np.abs(GRMS(result[2])-telem["grms"][2])<1e-9)
		print("Flags:",grms_check,skewness_check,kurtosis_check,sum_check,clearance_check,crest_check,p2p_check)
		#print(x.shape)
		x = x - np.mean(x)
		#print(mac,x)
		plt.plot(x,'-')
		#continue
	
	plt.legend(list(measurement_map.keys()))


	# import json

	# with open("anil_sync_measurement.json","w+") as f:
	# 	json.dump(measurement_map,f)

	plt.show()


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
