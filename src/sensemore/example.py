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


# #Take measurement from a specific device in the network

mac = 'CA:B8:31:00:00:55'

accelerometer_range = "16G"
sampling_frequency = 12800
sample_size = 50000

measurement_result = wired_network.measure(mac,accelerometer_range,sampling_frequency,sample_size)

result_acc_x = measurement_result[0]
result_acc_y = measurement_result[1]
result_acc_z = measurement_result[2]

plt.plot(result_acc_x)
plt.show()
# """
# Also there are telemetries calculated in wired, we can also take it by calling get_all_telemetry
# """

telemetries = wired_network.get_all_telemetry(mac)
print(telemetries)
exit()

"""
# With numpy and matplotlib, plot the accelerometer raw data
import numpy as np
from matplotlib import pyplot as plt

plt.plot(result_acc_x)
plt.show()
"""

self = wired_network




device_id = self.device_map[mac].user_defined_id
print("device_id:",device_id)
#Check the dictionaries
acc_index = SMWiredPy.acc_range_dict[accelerometer_range]
freq_index = SMWiredPy.sampling_frequency_dict[str(sampling_frequency)]
coef = SMWiredPy.SMWired.accelerometer_coefficients[acc_index]

#Below are all in bytes except sampling size, so no need to convert we already checked it
data = [acc_index,freq_index,*tuple(sample_size.to_bytes(4, "little"))]


v = wired_network.get_version(mac)
print("version request:",v)


def array_to_acc(arr,length,coef):
	out = [[0]*(length//6),[0]*(length//6),[0]*(length//6)]
	iter = 0
	for it in range(0,length,6):
		one_packet = arr[it:it+6]
		# new_acc->x = prev_acc.z; 					//This is now Sensor X
		# new_acc->y = prev_acc.x;					//This is now Sensor Y
		# new_acc->z = (int16_t)-1 * prev_acc.y;	//This is now Sensor Z
		# Apply wired rotation
		x = int.from_bytes(one_packet[4:6],byteorder='little',signed=True) * coef
		y = int.from_bytes(one_packet[0:2],byteorder='little',signed=True) * coef 
		z = -1 * int.from_bytes(one_packet[2:4],byteorder='little',signed=True) * coef

		out[0][iter] = x
		out[1][iter] = y
		out[2][iter] = z
		iter += 1

	return out


no_expected_packet = int(np.ceil(sample_size / 40))
packet_counter = 0
reverse_sample_size = sample_size*6

from matplotlib.animation import FuncAnimation
from random import randrange
import numpy as np
from numpy.core.fromnumeric import size


figure, ax = plt.subplots(nrows=4, ncols=1, sharex=False)
line1, = ax[0].plot([0,0],[1e6,-1e6], 'r',label='acc-x')
ax[0].set_ylabel("$Accelerometer_{X}$",rotation=90)
line2, = ax[1].plot([0,0],[1e6,-1e6], 'b',label='acc-y')
ax[1].set_ylabel("ACC-Y")
line3, = ax[2].plot([0,0],[1e6,-1e6], 'g',label='acc-z')
ax[2].set_ylabel("ACC-Z")
line4, = ax[3].plot([0,0],[1e6,-1e6], 'g',label='acc-z')
ax[3].set_ylabel("FFT-Z")



x_data,y_data = [],[]
y1,y2,y3 = [],[],[]

it = 0
def update(frame):
	global it,reverse_sample_size

	packet = self.get_message(timeout=100)
	assert(packet != None)
	expected_data_len = 240 if reverse_sample_size > 240 else reverse_sample_size
	packet_measurement = packet.data[1:1+expected_data_len]
	reverse_sample_size -= expected_data_len
	acc = array_to_acc(packet_measurement,expected_data_len,coef)

	x_data.extend(range(it,it+len(acc[0])))
	y1.extend(acc[0])
	y2.extend(acc[1])
	y3.extend(acc[2])
	#print(len(x_data),len(y1),print(x_data))
	
	it += len(acc[0])
	
	show_x1 = x_data[frame*40 : (frame+30)*40]
	show_y1 = y1[frame*40 : (frame+20)*40]
	#print(len(y1),len(show_y1))

	line1.set_data(x_data, y1)
	line2.set_data(x_data, y2)
	line3.set_data(x_data, y3)
 
 
	# ffty3 = np.fft.fft(y3)/len(y3)          # Normalize amplitude
	# ffty3 = ffty3[range(int(len(y3)/2))] # Exclude sampling frequency
	# tpCount     = len(y1)
	# values      = np.arange(int(tpCount/2))
	# timePeriod  = tpCount/(sampling_frequency)
	# frequencies = values/timePeriod
	# #axis[3].plot(frequencies, 20*np.log10(abs(fourierTransform)))
	# line4.set_data(frequencies, 20*np.log10(abs(ffty3)))

	# y1[:-1] = y1[1:]
	# y2[:-1] = y2[1:]
	# y3[:-1] = y3[1:]

	
	ax[0].relim()
	ax[0].autoscale_view()
	ax[1].relim()
	ax[1].autoscale_view()
	ax[2].relim()
	ax[2].autoscale_view()
	ax[3].relim()
	ax[3].autoscale_view()

	return line1,line2,line3,line4



#Check write!
self.write(device_id, SMWiredPy.SMCOM_WIRED_MESSAGES.START_STREAM_MEASUREMENT.value, data, len(data))

animation = FuncAnimation(figure, update, interval=1,blit=False)
plt.show()

#calculate end amount and give also additional time
#expected_timeout = sample_size/freq + (sample_size*1)
#for each packet calculate timeout
#data_packet = self.__data_queue.get(timeout = 100)

# no_expected_packet = int(np.ceil(sample_size / 40))
# print("No expected:",no_expected_packet)
# reverse_sample_size = sample_size*6
# it = 0




# for _ in range(no_expected_packet):
# 	packet = self.get_message(timeout=100)
# 	#or WIRED_MESSAGE_STATUS(packet.data[0]) != WIRED_MESSAGE_STATUS.SUCCESS
# 	if(packet != None):
# 		#Do not put the same recovery data twice, check it
# 		expected_data_len = 240 if reverse_sample_size > 240 else reverse_sample_size
# 		#print(_,"our :",it,"got:",int.from_bytes(packet.data[0:1],byteorder='little',signed=False))
# 		packet_measurement = packet.data[1:1+expected_data_len]
# 		reverse_sample_size -= expected_data_len
# 		print("Packet:",_,array_to_acc(packet_measurement,expected_data_len,coef))
		
# 	else:
# 		print("packet none!")
# 		break


