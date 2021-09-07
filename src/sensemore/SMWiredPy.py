from os import read, write
import serial
import threading
import time
import queue
import atexit
from enum import Enum
from sys import argv, stdout
from argparse import ArgumentParser

#Before importing SMComPy check the .so or .dll file!
import SMComPy
WIRED_APPLICATION_BAUDRATE = 115200
WIRED_FIRMWARE_UPDATE_BAUDRATE = 1000000
WIRED_MAX_DEVICE = 12 # Max allowed device number in the network!
PORT = "/dev/ttyUSB0" # Default port for linux
WIRED_FIRMWARE_MAX_RETRY_FOR_ONE_PACKET = 5
debug__ = False


__VERSION__ = "1.0.0"


class SMCom_version():
	def __init__(self, major, minor, patch):
		self.major = int(major)
		self.minor = int(minor)
		self.patch = int(patch)

	def __le__(self, other):
		self_ver = 10000 * self.major + 100 * self.minor + self.patch
		other_ver = 10000 * other.major + 100 * other.minor + other.patch
		return self_ver <= other_ver

	def __repr__(self):
		return f"{self.major}.{self.minor}.{self.patch}"

class Wired():
	def __init__(self, mac, version, user_id):
		self.mac = mac
		self.user_defined_id = user_id
		if isinstance(version, str):
			major, minor, patch = version.split('.')
		else:
			patch, minor, major = tuple(version)

		self.version = SMCom_version(major, minor, patch)

	def __eq__(self,other):
		return self.mac == other.mac

	def __ne__(self, other):
		return not self.__eq__(other)

	def __hash__(self):
		return hash(self.mac)

	def get_version(self):
		return self.version


class SMCOM_WIRED_MESSAGES(Enum):
	#------------- Bootloader Messages ----------------
	ENTER_FIRMWARE_UPDATER_MODE     = 0
	FIRMWARE_PACKET_START           = 1 #!< Application side should not get this message! only from the bootloader
	FIRMWARE_PACKET                 = 2	#!< Application side should not get this message! only from the bootloader
	FIRMWARE_PACKET_END             = 3
	#------------- Application Messages ----------------
	GET_VERSION 					= 10
	AUTO_ADDRESSING_INIT 			= 11
	AUTO_ADDRESSING_SET_NEW_ID 		= 12
	START_BATCH_MEASUREMENT 		= 13
	GET_BATCH_MEASUREMENT 			= 14
	GET_CLEARANCE					= 15
	GET_CREST 						= 16
	GET_GRMS 						= 17
	GET_KURTOSIS 					= 18
	GET_SKEWNESS 					= 19
	GET_BATCH_MEASUREMENT_CHUNK 	= 20
	AUTO_ADDRESSING_INTEGRITY_CHECK = 21
	GET_ALL_TELEMETRY 				= 22
	GET_VRMS						= 23
	GET_PEAK						= 24
	GET_SUM							= 25
	GET_LAST_MEASUREMENT_STATUS		= 26

class WIRED_MESSAGE_STATUS(Enum):
	ERROR               	= 0 	#!< [0] If the corresponding msg_handler fails put 0 for result status as an error, maybe additional message explanation
	SUCCESS             	= 1 	#!< [1] If everything is okay handler sends 1 to indicate message is handled succesfully
	TIMEOUT             	= 2		#!< [3] If the message handler sees a timeout error send this
	WRONG_MESSAGE       	= 3 	#!< [5] If incoming message is broken or data is missing
	BROKEN_PACKET       	= 4
	NO_MEASUREMENT 			= 5
	INVALID_MEASUREMENT		= 6
	BROKEN_MEASUREMENT 		= 7
	MEASUREMENT_TIMEOUT 	= 8
	FLASH_ERASE_ERROR		= 9
	FLASH_WRITE_ERROR		= 10
	FLASH_READ_ERROR		= 11
	NO_MEMORY				= 12
	ACCELEROMETER_ERROR		= 13



class WIRED_ACCELEROMETER_RANGE(Enum):
	RANGE_2G    = 1,
	RANGE_4G    = 2,
	RANGE_8G    = 3,
	RANGE_16G   = 4

acc_range_dict = {
	"2G":1,
	"4G":2,
	"8G":3,
	"16G":4,
}

sampling_frequency_dict = {
	"800":5,
	"1600":6,
	"3200":7,
	"6400":8,
	"12800":9
}

class SMComPyPacket:
	def __init__(self,smcom_cpp_packet):
		self.data_len = smcom_cpp_packet.data_len
		self.receiver_id = smcom_cpp_packet.receiver_id
		self.transmitter_id = smcom_cpp_packet.transmitter_id
		self.message_type = smcom_cpp_packet.message_type
		self.message_id = smcom_cpp_packet.message_id
		self.data = smcom_cpp_packet.data

	def verify_packet(self,message_id=None,transmitter_id=None,receiver_id = None,**kwargs):
		ret = True
		if(ret and message_id != None):
			ret = ret and self.message_id == message_id

		if(ret and transmitter_id != None):
			ret = ret and self.transmitter_id == transmitter_id

		if(ret and receiver_id != None):
			ret = ret and self.receiver_id == receiver_id

		return ret

class SMWired(SMComPy.SMCOM_PUBLIC):
	accelerometer_coefficients= [(2*2)/(1<<16), (2*2)/(1<<16), (4*2)/(1<<16), (8*2)/(1<<16), (16*2)/(1<<16)]

	def __init__(self, port = PORT, configure_network = 'auto', max_device_number = WIRED_MAX_DEVICE):
		"""
			The global SMWired object opens an usb port and builds the network
			if network parameter is set to 'auto', it tries to scan network automatically and finds available wired devices
			if network is set to list of mac addresses such as ['CA:B8:31:xx:XX:01','CA:B8:31:xx:XX:02','CA:B8:31:xx:XX:03', etc] it tries to find only this devices in the network
				later scan function can be called to find available devices
			'SMWired.scan' function to communicate with the network
		"""
		self.__master_id = 13 #master id defined by us never change it!
		super().__init__(self.__master_id)
		self.transmitter_id = self.__master_id
		self.__data_queue = queue.Queue()
		self.__mutex = threading.Lock()
		self.__mutex_timeout = 10
		self.__continue_thread = True

		try:
			self.__ser = serial.Serial(port,baudrate=WIRED_APPLICATION_BAUDRATE)
		except:
			self.__ser = None
			print("Serial port cannot be opened, please check usb is placed correctly!")
			exit()
			return

		atexit.register(self.__del__)
		

		self.__listener_thread = threading.Thread(target=self.__thread_func__, daemon=True)
		self.__listener_thread.start()
		time.sleep(0.05)
		self.device_map = {}

		if(configure_network == 'auto'):
			self.scan(max_device_number)
		if(isinstance(configure_network,list)):
			if(self.___scan_wired_mac_list(configure_network) == False):
				exit()

	def __del__(self):
		self.__continue_thread = False
		#Try to get mutex immediately or wait 
		if(self.__mutex.acquire(timeout=1)):
			self.__mutex.release()
		self.__ser.close()

	def __thread_func__(self):
		"""
			thread function that checks available messages in the network, if any! listener function invokes __rx_callback__ if there is a message in the network
		"""
		while self.__continue_thread:
			x = self.listener()
			time.sleep(10*1e-6)

	def __write__(self, buffer, length):
		"""
			overloaded __write__ function, inherited from SMCom
		"""
		if buffer == None or buffer == [] or buffer == "" or buffer == b'':
			return SMComPy.SMCOM_STATUS_FAIL
		elif type(buffer) == str:
			buffer = bytes(buffer, "utf-8")
		elif type(buffer) == int:
			buffer = bytes([buffer])
		elif type(buffer) == list or type(buffer) == tuple:
			if type(buffer[0]) == int:
				buffer = bytes(buffer)
			elif type(buffer[0]) == str:
				buffer = bytes("".join(buffer), "utf-8")

		if(self.__mutex.acquire(blocking=True, timeout=self.__mutex_timeout)):
			self.__ser.write(buffer)
			self.__mutex.release()
			return SMComPy.SMCOM_STATUS_SUCCESS
		else:
			print("Got no mutex!")
			exit()

		return SMComPy.SMCOM_STATUS_TIMEOUT

	def __rx_callback__(self, event, status, packet):
		"""
			overloaded __rx_callback__ function, inherited from SMCom
		"""
		#print("rx callback:",status)
		if(status != SMComPy.SMCOM_STATUS_SUCCESS):
			print("Error occured rx callback!")
			self.__ser.flush()
			return
		temp_packet = SMComPyPacket(packet) #Convert to python so we can have better interface
		self.__data_queue.put(temp_packet)

	def __tx_callback__(self, event, status, packet):
		"""
			overloaded __tx_callback__ function, inherited from SMCom
		"""
		pass

	def __available__(self):
		"""
			overloaded __available__ function, inherited from SMCom
		"""
		avlb = 0
		#if(self.__mutex.acquire(blocking=True,timeout=self.__mutex_timeout)):
			#print("__avaiable__ got mutex")
		avlb = self.__ser.inWaiting()
		#	self.__mutex.release()
		return avlb

	def __read__(self, length):
		"""
			overloaded __read__ function, inherited from SMCom
		"""
		buffer,temp,pair = [], [], SMComPy.SMCom_Pair()
		if(self.__mutex.acquire(blocking=True,timeout=self.__mutex_timeout)):
			temp = self.__ser.read(length)
			self.__mutex.release()

		if(len(temp) == 0):
			return None
		#print("__rx__ temp:",temp)
		pair.vec = list(temp)
		pair.status = SMComPy.SMCOM_STATUS_SUCCESS
		return pair

	def __get_message(self,timeout):
		"""
			Instead of handling exception of queue, this function try to take data, otherwise returns None
		"""
		try:
			packet = self.__data_queue.get(timeout=timeout)
			return packet
		except queue.Empty:
			return None

	def ___scan_wired_mac_list(self,ls):
		"""
			Take list of mac addresses and tries to assign id and check their accessibility
		"""
		ls = set(ls) # allow set or list, convert to set so we don't want to iterate over the same mac
		for mac in ls:
			mac_as_byte = [int(k, base = 16) for k in mac.split(':')]
			user_defined_id = self.__find_available_id()
			write_ret = self.__assign_new_id(mac, user_defined_id)
			if(write_ret !=  SMComPy.SMCOM_STATUS_SUCCESS):
				print("USB port write problem")
				return False

			#This message changes the assigned id, use it with care!
			write_ret = self.write(user_defined_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value, [0,0,0,0,0], 5)
			if(write_ret !=  SMComPy.SMCOM_STATUS_SUCCESS):
				print("USB port write problem")
				return False

			try:
				packet = self.__data_queue.get(timeout=1)
				data = packet.data
				inc_mac = data[:-3]
				inc_version = data[6:]
				if(inc_mac != mac_as_byte):
					print("Wrong id assigned to device!, integrity check failed")
					continue
				self.device_map[mac] = Wired(mac,inc_version,user_defined_id)
				write_ret = self.__assign_new_id(mac, user_defined_id) # So device is on the entwork, assign it again
				return True
			except queue.Empty:
				print("No device found ",mac)
				return False

				#print("Cannot add device %s, not in the network or connection problem"%mac)

	def get_version(self:object, mac:str, timeout:int = 1):
		"""
		Takes mac address of the device and returns its version in format (MAJOR.MINOR.PATCH)(Ex. 1.0.12)
		If cannot communicate via device, returns None
		"""

		user_defined_id = self.device_map[mac].user_defined_id
		write_ret = self.write(user_defined_id, SMCOM_WIRED_MESSAGES.GET_VERSION.value, [], 0)
		if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
			return None
		packet = self.__get_message(timeout=timeout)

		if(packet == None):
			return None

		SMComPy_version = packet.data
		version = f"{SMComPy_version[2]}.{SMComPy_version[1]}.{SMComPy_version[0]}"
		return version

	def __get_mac_address(self, device_id, timeout = 3):
		"""
		Takes receiver id as argument and returns mac address of the device with given id as string in format
		(XX:XX:XX:XX:XX:XX) (15 or 255 are reserved as public id).
		Not available for users, only used for development
		"""
		write_ret = self.write(device_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value, [0,0,0,0,0], 5)
		if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
			return write_ret

		packet = self.__data_queue.get(timeout = timeout)

		data = packet.data
		data = tuple(data[:-3])
		return "%02X:%02X:%02X:%02X:%02X:%02X"%data

	def __assign_new_id(self, mac_address, user_id):
		"""
		Takes mac adress and id to be assigned to the given mac address as arguments assigns the
		receiver id to the device which has mac address same as given and return None.
		"""
		if(type(mac_address) == str):
			mac_address = [int(x, base = 16) for x in mac_address.split(':')]
		data = [user_id,*mac_address]
		write_ret = self.write(SMComPy.PUBLIC_ID_4BIT.value, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_SET_NEW_ID.value, data, len(data))
		return write_ret

	def start_batch_measurement(self, mac, acc, freq, sample_size, notify_measurement_end = True):
		if(sample_size <= 0 or sample_size > 1000000 or (str(freq) not in sampling_frequency_dict.keys()) or (str(acc) not in acc_range_dict.keys()) ):
			#Return arg error here
			return None

		device_id = self.device_map[mac].user_defined_id
		#Check the dictionaries
		acc_index = acc_range_dict[acc]
		freq_index = sampling_frequency_dict[str(freq)]
		#Below are all in bytes except sampling size, so no need to convert we already checked it
		data = [acc_index,freq_index,*tuple(sample_size.to_bytes(4, "little")), notify_measurement_end]

		#Check write!
		self.write(device_id, SMCOM_WIRED_MESSAGES.START_BATCH_MEASUREMENT.value, data, len(data))

		if(notify_measurement_end):
			#calculate end amount and give also additional time
			expected_timeout = sample_size/freq + (sample_size*1)
			data_packet = self.__data_queue.get(timeout = expected_timeout)
			return (data_packet.data[0] == WIRED_MESSAGE_STATUS.SUCCESS.value)

		return True
	
	def start_sync_batch_measurement(self,acc,freq,sample_size):
		if(sample_size <= 0 or sample_size >= 1000000 or (str(freq) not in sampling_frequency_dict.keys()) or (str(acc) not in acc_range_dict.keys()) ):
			#Return arg error here
			return None
		acc_index = acc_range_dict[acc]
		freq_index = sampling_frequency_dict[str(freq)]

		data = [acc_index,freq_index,*tuple(sample_size.to_bytes(4, "little")), 0]
		#Check write!
		self.write(SMComPy.PUBLIC_ID_4BIT.value, SMCOM_WIRED_MESSAGES.START_BATCH_MEASUREMENT.value, data, len(data))
		#calculate end amount and give also additional time for erasing flash
		expected_timeout = sample_size/freq + (sample_size*4e-5) + 1
		print("Expected timeout:",expected_timeout)
		time.sleep(expected_timeout)
		return True

	def check_last_measurement_status(self,mac):
		device_id = self.device_map[mac].user_defined_id
		self.write(device_id, SMCOM_WIRED_MESSAGES.GET_LAST_MEASUREMENT_STATUS.value,[],0)
		packet = self.__get_message(timeout=2)
		print("cechking last meas:",packet.data)


	def read_measurement(self, mac, sample_size, coefficient = 0, timeout = 10):
		device_id = self.device_map[mac].user_defined_id

		current_byte_offset = 0
		measurement_byte_size = sample_size * 6 #Convert to bytes
		reverse_byte_size = (sample_size*6) # Counts by reverse, when it hits to zero we must get all the measurement
		expected_data_len = min(240,reverse_byte_size) # For the last package this may not be 240, other than that it will be 240 always!
		data_recovery_list = []

		tmp = {}
		tmp["byte_offset"] = 0
		tmp["data_len"] = measurement_byte_size
		data_recovery_list.append(tmp)

		raw_measurement_data = [0]*(sample_size*6)
		from math import ceil

		_max_retry_for_read = 5
		retry = 0
		last_packet_size = measurement_byte_size%240
		last_packet_byte_offset = (measurement_byte_size - last_packet_size)/240 + (last_packet_size != 0)
		if(last_packet_size == measurement_byte_size):
			last_packet_byte_offset = 0

		while(len(data_recovery_list) > 0 and retry < _max_retry_for_read):
			tmp = data_recovery_list.pop()
			tmp_byte_offset = tmp["byte_offset"]
			tmp_data_len = tmp["data_len"]
			
			current_byte_offset = tmp_byte_offset

			data = [*tuple(tmp_byte_offset.to_bytes(4, "little")),*tuple(tmp_data_len.to_bytes(4, "little"))]
			self.write(device_id, SMCOM_WIRED_MESSAGES.GET_BATCH_MEASUREMENT_CHUNK.value, data, len(data))
			retry += 1
			#iterate over all expected packets, even though they could be broken we will try to read
			no_expected_packet = ceil(tmp_data_len / 240)
			for _ in range(no_expected_packet):
				packet = self.__get_message(timeout=3)
				if(packet == None or WIRED_MESSAGE_STATUS(packet.data[0]) != WIRED_MESSAGE_STATUS.SUCCESS):
					#Do not put the same recovery data twice, check it
					tmp = {}
					tmp["byte_offset"] = current_byte_offset
					tmp["data_len"] = 240 if current_byte_offset != last_packet_byte_offset else last_packet_size
					data_recovery_list.append(tmp)
					current_byte_offset += expected_data_len
					if(len(data_recovery_list) >= 30):
						print("Lost the connection")
						return None
					continue
				#Got the packet and its status ok!
				retry = 0
				packet_data_len = packet.data[1]
				packet_measurement = packet.data[2:2+expected_data_len]

				raw_measurement_data[current_byte_offset:current_byte_offset+packet_data_len] = packet_measurement
				reverse_byte_size -= packet_data_len
				expected_data_len = 240 if current_byte_offset != last_packet_byte_offset else last_packet_size
				current_byte_offset += packet_data_len

		if(retry >= _max_retry_for_read):
			print("------------- Cannot read measurement : retry failed ------------ ")
			exit()
			return None
		
		if(reverse_byte_size != 0):
			print("------------- Cannot read measurement : measurement data broken ------------ ")
			exit()
			return None

		measurement_data = [[0]*sample_size,[0]*sample_size,[0]*sample_size]
		#Handle coefficient, default is zero which is not multiplied by coefficient!
		coef = coefficient
		if(coef == 0):
			coef = 1 #multiply by itself

		iter = 0
		for it in range(0,sample_size*6,6):
			one_packet = raw_measurement_data[it:it+6]
			measurement_data[0][iter] = int.from_bytes(one_packet[0:2],byteorder='little',signed=True)*coef
			measurement_data[1][iter] = int.from_bytes(one_packet[2:4],byteorder='little',signed=True)*coef
			measurement_data[2][iter] = int.from_bytes(one_packet[4:6],byteorder='little',signed=True)*coef
			iter += 1

		return measurement_data

	def x_read_measurement(self, mac, sample_size, coefficient = 0, timeout = 10):
		#Wait for all packets
		byte_offset = 0 # Start from the beginning
		measurement_data_len = sample_size * 6 #Convert to bytes

		device_id = self.device_map[mac].user_defined_id
		data = [*tuple(byte_offset.to_bytes(4, "little")),*tuple(measurement_data_len.to_bytes(4, "little"))]

		#Check write!
		self.write(device_id, SMCOM_WIRED_MESSAGES.GET_BATCH_MEASUREMENT_CHUNK.value, data, len(data))

		measurement_data = [[0]*sample_size,[0]*sample_size,[0]*sample_size]
		raw_measurement_data = [[0]*(sample_size*6)]

		data_recovery_list = []
		reverse_byte_size = (sample_size*6) # Counts by reverse, when it hits to zero we must get all the measurement

		expected_data_len = min(240,reverse_byte_size) # For the last package this may not be 240, other than that it will be 240 always!

		raw_it = 0
		current_byte_offset = 0
		while(len(raw_measurement_data) < measurement_data_len):
			packet = self.__get_message(timeout=10)
			if(packet == None or packet.status != SMComPy.SMCOM_STATUS_SUCCESS):
				tmp = {}
				tmp["byte_offset"] = current_byte_offset
				tmp["data_len"] = expected_data_len
				data_recovery_list.append(tmp)
				continue
			
			raw_data = packet.data
			measurement_status = WIRED_MESSAGE_STATUS(raw_data[0])
			if(measurement_status != WIRED_MESSAGE_STATUS.SUCCESS.value):
				tmp = {}
				tmp["byte_offset"] = current_byte_offset
				tmp["data_len"] = expected_data_len
				data_recovery_list.append(tmp)
				continue

			packet_data_len = raw_data[1]
			packet_measurement = raw_data[2:2+expected_data_len]
			raw_measurement_data[raw_it:raw_it+expected_data_len] = packet_measurement
			raw_it += packet_data_len
			current_byte_offset = len(raw_measurement_data)
			expected_data_len = min(240,reverse_byte_size)
			reverse_byte_size -= packet_data_len
			print("current byte offset:",current_byte_offset)

		retry = 0
		while(len(data_recovery_list) > 0 and retry < 5):
			tmp = data_recovery_list[-1]
			print("Lost packet in recovery list:",tmp)
			tmp_byte_offset = tmp["byte_offset"]
			tmp_data_len = tmp["data_len"]
			data = [*tuple(tmp_byte_offset.to_bytes(4, "little")),*tuple(tmp_data_len.to_bytes(4, "little"))]
			self.write(device_id, SMCOM_WIRED_MESSAGES.GET_BATCH_MEASUREMENT_CHUNK.value, data, len(data))
			packet = self.__get_message(timeout=10)
			if(packet != None):
				#Got the data!
				raw_data = packet.data
				packet_data_len = raw_data[1]
				packet_measurement = raw_data[2:2+tmp_data_len]
				raw_measurement_data[tmp_byte_offset:tmp_byte_offset+tmp_data_len] = packet_measurement
				


		#Handle coefficient, default is zero which is not multiplied by coefficient!
		coef = coefficient
		if(coef == 0):
			coef = 1 #multiply by itself

		iter = 0
		for it in range(0,sample_size*6,6):
			one_packet = raw_measurement_data[it:it+6]
			measurement_data[0][iter] = int.from_bytes(one_packet[0:2],byteorder='little',signed=True)*coef
			measurement_data[1][iter] = int.from_bytes(one_packet[2:4],byteorder='little',signed=True)*coef
			measurement_data[2][iter] = int.from_bytes(one_packet[4:6],byteorder='little',signed=True)*coef
			iter += 1

		print("Returning measurement")
		return measurement_data

	def measure(self, mac, acc, freq, sample_size, timeout=10):
		if(self.start_batch_measurement(mac, acc, freq, sample_size, notify_measurement_end=True) == True):
			coef = self.accelerometer_coefficients[acc_range_dict[acc]]
			return self.read_measurement(mac,sample_size,coefficient= coef,timeout = timeout)
		else:
			print("Measurement failed")
			return None

	def measure_sync(self, acc, freq, sample_size, timeout=10):
		
		if(self.start_sync_batch_measurement(acc, freq, sample_size) == False):
			print("Measurement failed")
			return False
		
		coef = self.accelerometer_coefficients[acc_range_dict[acc]]
		meas = {}
		for mac in self.device_map:
			meas[mac] = self.read_measurement(mac,sample_size,coefficient= coef,timeout = timeout)
		
		return meas
		
	def get_all_telemetry(self, mac, timeout = 30):
		device_id = self.device_map[mac].user_defined_id
		#Check write!
		self.write(device_id, SMCOM_WIRED_MESSAGES.GET_ALL_TELEMETRY.value, [], 0)
		#Check also queue data receiver id!, maybe not desired message
		data = self.__data_queue.get(timeout = timeout).data

		"""
			Telemetry message response :
				typedef struct resp_get_all_telemetry{
					STATUS_SENSEWAY_WIRED status;
					uint16_t temperature;
					uint32_t frequency_calibration;
					double clearance[3];
					double crest[3];
					double grms[3];
					double kurtosis[3];
					double skewness[3];
					//Included in v1.0.9
					double vrms[3];
					double peak[3];
					double sum[3];
					//Included in v1.0.13
					double peak_to_peak[3];
				}__attribute__((packed)) resp_get_all_telemetry;
		"""
		status = WIRED_MESSAGE_STATUS(data[0])
		if(status  != WIRED_MESSAGE_STATUS.SUCCESS):
			print("Telemetry read error")
			return None

		temperature = int.from_bytes(data[1:3],"little",signed=False)
		calibrated_frequency = int.from_bytes(data[3:7],"little",signed=False)
		telemetries = data[7:]

		import struct
		def convert_byte_list_to_double(bl):
			return struct.unpack('d',bytes(bl))[0]

		def byte_list_to_double_list(bl,index):
			#index must be integer order 0,1,2,3 etc
			i = index*24
			return [convert_byte_list_to_double(bl[i:i+8]),
					convert_byte_list_to_double(bl[i+8:i+16]),
					convert_byte_list_to_double(bl[i+16:i+24])]

		dev_version = self.device_map[mac].version

		clearance = byte_list_to_double_list(telemetries,0)
		crest = byte_list_to_double_list(telemetries,1)
		grms = byte_list_to_double_list(telemetries,2)
		kurtosis = byte_list_to_double_list(telemetries,3)
		skewness = byte_list_to_double_list(telemetries,4)

		telems = {
			"temperature":temperature/100,
			"calibrated_frequency":calibrated_frequency,
			"clearance":clearance,
			"crest":crest,
			"grms":grms,
			"kurtosis":kurtosis,
			"skewness":skewness
		}

		if dev_version.patch >= 9:
			vrms = byte_list_to_double_list(telemetries,5)
			peak = byte_list_to_double_list(telemetries,6)
			sum = byte_list_to_double_list(telemetries,7)
			telems["vrms"] = vrms
			telems["peak"] = peak
			telems["sum"] = sum

		if dev_version.patch >= 13:
			peak_to_peak = byte_list_to_double_list(telemetries,8)
			telems["peak_to_peak"] = peak_to_peak

		return telems

	def firmware_update(self, mac, bin_file_address, timeout = 10):
		"""
		Takes mac_address, address of binary file, and timeout(default 10) and return SMComPy_Status_t
		or WIRED_MESSAGE_STATUS type as int.
		"""
		bin_file = open(bin_file_address, "rb")
		packets = []
		while True:
			temp = bin_file.read(240)
			if temp == b'':
				break
			packets.append(list(temp))
		packets[-1] += [0] * (240 - len(packets[-1]))

		bin_file_size = 240*(len(packets)-1) + len(packets[-1])
		if bin_file_size == 0:
			print("Empty File")
			return

		if(isinstance(mac,str)):
			mac_as_byte = [int(x,16) for x in mac.split(':')]

		receiver_id = self.device_map[mac].user_defined_id
		enter_message = [*mac_as_byte]
		write_ret = self.write(receiver_id, SMCOM_WIRED_MESSAGES.ENTER_FIRMWARE_UPDATER_MODE.value, enter_message, len(enter_message))
		time.sleep(0.01)
		self.__ser.baudrate = WIRED_FIRMWARE_UPDATE_BAUDRATE
		enter_mac_return = self.__data_queue.get(timeout = timeout).data
		if enter_mac_return != mac_as_byte:
			return SMComPy.SMCOM_STATUS_FAIL
		try:
			if(write_ret != SMComPy.SMCOM_STATUS_SUCCESS):
				return write_ret

			print("Device %s entered firmware updater mode"%mac)
			bootloader_id = 12 # This is predefined id for bootloader
			no_packets = len(packets) - 1

			start_message = [*tuple(bin_file_size.to_bytes(4, "little")),*tuple(no_packets.to_bytes(4, "little")), *mac_as_byte]

			write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET_START.value, start_message, len(start_message))

			if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
				print("Cannot send message")
				return write_ret

			read_ret = self.__data_queue.get(timeout = timeout).data
			start_mac_return = read_ret[1:]
			status = read_ret[0]

			if mac_as_byte != start_mac_return or status != WIRED_MESSAGE_STATUS.SUCCESS.value:
				print("Wired error")
				return SMComPy.SMCOM_STATUS_FAIL

			for packet_no in range(no_packets + 1):
				retry = 0
				wired_packet_no = packet_no + 1
				success = False
				data_packet = [*tuple(packets[packet_no]), *mac_as_byte, *tuple(wired_packet_no.to_bytes(2, "little"))]
				while retry < WIRED_FIRMWARE_MAX_RETRY_FOR_ONE_PACKET:
					retry += 1
					write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET.value, data_packet, len(data_packet))
					resp_msg_data = self.__data_queue.get(timeout = timeout).data
					# print(f"retry: {retry} for packet_no {wired_packet_no}")
					if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
						# print("write error:",write_ret)
						continue
					read_ret = resp_msg_data[0]
					packet_mac_return = resp_msg_data[1:7]     # resp data[0] is wired status and 1: is mac address returned wrt the msg
					ret_packet_no = resp_msg_data[7:]
					if ret_packet_no[0] != wired_packet_no:
						return SMComPy.SMCOM_STATUS_FAIL
					if mac_as_byte != packet_mac_return or read_ret != WIRED_MESSAGE_STATUS.SUCCESS.value:
						# print("data error:",read_ret)
						continue
					success = True
					break

				if not success:
					return SMComPy.SMCOM_STATUS_FAIL

			#Data transmission ended successfully
			print("All firmware data sent")

			write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET_END.value, mac_as_byte, len(mac_as_byte))
			if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
				return write_ret

			resp_end = self.__data_queue.get(timeout = timeout).data
			read_ret = resp_end[0]
			end_mac_return = resp_end[1:]
			if mac_as_byte != end_mac_return or read_ret != WIRED_MESSAGE_STATUS.SUCCESS.value:
				return SMComPy.SMCOM_STATUS_FAIL
			self.__ser.baudrate = WIRED_APPLICATION_BAUDRATE
			time.sleep(12)

			receiver_id = self.device_map[mac].user_defined_id
			self.__assign_new_id(mac,receiver_id)
			print("Device '%s' updated to version:"%(mac), self.get_version(mac))
		except Exception as e:
			print(e)
			print("An error occurred while firmware update")
		finally:
			self.__ser.baudrate = WIRED_APPLICATION_BAUDRATE

	def __find_available_id(self):
		flag = 0 #16bit data, since max device is smaller than 16 we can use it safely
		#Now set the bits if we have a device at that bit
		for x in self.device_map.values():
			flag |= (0x01 << x.user_defined_id)
		#Find the first zero bit
		for i in range(len(self.device_map)):
			if( ( (flag>>i) & 0x01) == 0):
				return i
		return len(self.device_map)

	def scan(self,max_device = WIRED_MAX_DEVICE, max_retry = 5, verbose = False):
		device_count = 0
		delay_offset = 150
		channel_delay = 100

		self.device_map = {}
		msg_start_adr = SMComPy.DEFAULT_ID_4BIT.value
		if len(self.device_map) == 0:
			msg_start_adr = SMComPy.PUBLIC_ID_4BIT.value

		retry = 0
		for retry in range(max_retry):
			device_count = 2*(max_device - len(self.device_map))
			wait_time = (device_count * channel_delay) + delay_offset
			wired_id = SMComPy.DEFAULT_ID_4BIT.value if retry != 0 else msg_start_adr

			msg_data = [*tuple(device_count.to_bytes(1, "little")), *tuple(delay_offset.to_bytes(2, "little")), *tuple(channel_delay.to_bytes(2, "little"))]

			#Just dummy loop to send correctly if serial fails!
			for _ in range(5):
				write_ret = self.write(wired_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value, msg_data, len(msg_data))
				if write_ret == SMComPy.SMCOM_STATUS_SUCCESS or write_ret == SMComPy.SMCOM_STATUS_PORT_BUSY:
					break
				else:
					return {}

			time.sleep(wait_time/1000)
			read_amount = self.__data_queue.qsize()
			for _ in range(read_amount):
				packet = self.__data_queue.get()
				if packet.message_id != SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value:
					break

				incoming_data = packet.data
				mac_adr = incoming_data[:6]
				mac_adr = "%02X:%02X:%02X:%02X:%02X:%02X"%tuple(mac_adr)
				version = incoming_data[6:]
				if(verbose):
					print("Wired device '%s' found"%mac_adr)
				_id = self.__find_available_id()
				write_ret = self.__assign_new_id(mac_adr, _id)
				if(write_ret == SMComPy.SMCOM_STATUS_SUCCESS):
					self.device_map[mac_adr] = Wired(mac_adr,version, _id)
				else:
					print("Cannot add device ",mac_adr)

			if(len(self.device_map) == max_device):
				break

		if(len(self.device_map) == 0 and verbose):
			print("No device found in network")
			return {}

		return self.device_map


	def integrity_check(self):
		if debug__ : print(f"Integrity check for {len(self.device_map)} devices")
		for i in self.device_map:
			device = self.device_map[i]
			mac_adr = [int(k, base = 16) for k in i.split(':')]
			retry = 0
			while True:
				write_ret = self.write(device.user_defined_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INTEGRITY_CHECK.value, mac_adr, len(mac_adr))
				if write_ret == SMComPy.SMCOM_STATUS_SUCCESS:
					try:
						read_ret = self.__data_queue.get(timeout = 1)
						if read_ret.data != mac_adr:
							if debug__: print("Unexpected mac!")
							retry += 1
							continue
						break
					except queue.Empty:
						if debug__: print(f"Erasing device with id {device.user_defined_id}, mac : {i}")
						self.assign_new_id(i, 14)
						time.sleep(.01)
						self.device_map.pop(i)

	def device_check(self,max_retry = 5):
		self.integrity_check()
		if len(self.device_map) == WIRED_MAX_DEVICE:
			return WIRED_MESSAGE_STATUS.SUCCESS
		self.scan(max_retry = max_retry)

	def get_available_devices(self):
		return list(self.device_map.keys())


def parser_function():
	parser = ArgumentParser(description = "SMComPy wired library to get measurements and update firmware")
	sub_parsers = parser.add_subparsers(help = 'sub-command help')

	parser.add_argument('-v','--version',action="version", version= __VERSION__)

	update_parser = sub_parsers.add_parser('update', help = 'Update firmware of the device connected to given port with given bin file')
	update_parser.set_defaults(which='update')
	update_parser.add_argument('-p','--port', type = str, help = "port address of the device (linux /dev/ttyUSBX, win32 COMX)",default='/dev/ttyUSB0')
	update_parser.add_argument('-f','--file',type = str, help = "address of the binary file containing the firmware update",default=None)
	update_parser.add_argument('-m','--mac',type = str, help = "address of the binary file containing the firmware update",default=None)

	measurement_parser = sub_parsers.add_parser('measure', help = 'Take measurements directly from command line')
	measurement_parser.set_defaults(which='measure')
	measurement_parser.add_argument('-m','--mac',type = str, help = "address of the binary file containing the firmware update",default=None)
	measurement_parser.add_argument('-p','--port'    ,type = str, help = "port address of the device (linux /dev/ttyUSBX, win32 COMX",default='/dev/ttyUSB0')
	measurement_parser.add_argument('-a','--acc'     ,type = str, help = "acceleration range: Possible args: 2G, 4G, 8G, 16G",default="16G")
	measurement_parser.add_argument('-f','--freq'    ,type = int, help = "sampling frequency: Possible args: 800, 1600, 3200, 6400, 12800",default=12800)
	measurement_parser.add_argument('-s','--sample'  ,type = int, help = "sampling size: Number of samples")
	measurement_parser.add_argument('-o','--output' ,type = str, help = "output file address which measurement data will be written",default=stdout)
	measurement_parser.add_argument('-t','--telem' ,action='store_true', help = "can be set or notset. if set, telemetries will be written at the beginning of the file")

	if(len(argv) <= 1):
		print("No argument is provided!")
		parser.print_help()
		exit()

	args = parser.parse_args()

	if(args.which == 'update'):
		mac = args.mac
		file = args.file
		network = SMWired(port = args.port,configure_network=[mac])
		network.firmware_update(mac,file)
	elif(args.which == 'measure'):
		mac = args.mac
		freq = args.freq
		acc = args.acc
		sample=args.sample
		network = SMWired(port = args.port,configure_network=[mac])
		meas = network.measure(mac,acc,freq,sample)
		ls = []
		for i in range(len(meas[0])*3):
			ls.append(meas[i%3][i//3])
		terminal = False
		if args.output == stdout:
			terminal = True
			f = stdout

		if not terminal:
			f = open(args.output, "w")
		if args.telem:
			telems = network.get_all_telemetry(mac)
			for i in telems.keys():
				print(f"{i} : {telems[i]}", file = f)
		for i in ls:
			print(i, file = f)
		if not terminal:
			f.close()

if __name__ == "__main__":
	parser_function()
