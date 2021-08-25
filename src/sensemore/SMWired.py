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
BAUD_RATE = 115200
WIRED_MAX_DEVICE = 12 # Max allowed device number in the network!
PORT = "/dev/ttyUSB0" # Default port for linux
WIRED_FIRMWARE_MAX_RETRY_FOR_ONE_PACKET = 5
debug__ = False

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
	def __init__(self, mac, version, id):
		self.mac = mac
		self.user_defined_id = id
		if isinstance(version, str):
			major, minor, patch = version.split('.')
		else:
			patch, minor, major = version
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

class WIRED_MESSAGE_STATUS(Enum):
	ERROR               = 0 	#!< [0] If the corresponding msg_handler fails put 0 for result status as an error, maybe additional message explanation
	SUCCESS             = 1 	#!< [1] If everything is okay handler sends 1 to indicate message is handled succesfully
	TIMEOUT             = 2		#!< [3] If the message handler sees a timeout error send this
	DATA                = 3		#!< [4] If we send data we will put first this result
	WRONG_MESSAGE       = 4 	#!< [5] If incoming message is broken or data is missing
	BROKEN_PACKET       = 5



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

class PySMComPacket:
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

	def __init__(self:SMWired, port:str = PORT, configure_network = 'auto'):
		"""
			The global SMWired object opens an usb port and builds the network
			if network parameter is set to 'auto', it tries to scan network automatically and finds available wired devices
			if network is set to list of mac addresses such as ['CA:B8:31:xx:XX:01','CA:B8:31:xx:XX:02','CA:B8:31:xx:XX:03', etc] it tries to find only this devices in the network
				later scan function can be called to find available devices
			'SMWired.scan' function to communicate with the network
		"""
		self.device_id = 13 #master id defined by us never change it!
		super().__init__(self.device_id)
		self.transmitter_id = id
		try:
			self.ser = serial.Serial(port, BAUD_RATE)
		except:
			self.ser = None
			print("Serial port cannot be opened, please check usb is placed correctly!")
			return

		atexit.register(self.__del__)
		
		self.data_queue = queue.Queue()
		self.mutex = threading.Lock()
		self.mutex_timeout = 10
		self.continue_thread = True
		
		self.listener_thread = threading.Thread(target=self.__thread_func__, daemon=True)
		self.listener_thread.start()
		time.sleep(0.05)
		self.device_map = {}

		if(configure_network == 'auto'):
			self.scan(WIRED_MAX_DEVICE)
		if(isinstance(configure_network,list)):
			self.___scan_wired_mac_list(configure_network)

	def __del__(self):
		self.continue_thread = False
		self.ser.close()

	def __thread_func__(self):
		"""
			thread function that checks available messages in the network, if any! listener function invokes __rx_callback__ if there is a message in the network
		"""
		while self.continue_thread:
			self.listener()
			time.sleep(0.01)
		
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

		if(self.mutex.acquire(blocking=True, timeout=self.mutex_timeout)):
			self.ser.write(buffer)
			self.mutex.release()
			return SMComPy.SMCOM_STATUS_SUCCESS
		else:
			print("Got no mutex!")

		return SMComPy.SMCOM_STATUS_TIMEOUT

	def __rx_callback__(self, event, status, packet):
		"""
			overloaded __rx_callback__ function, inherited from SMCom
		"""
		if(status != SMComPy.SMCOM_STATUS_SUCCESS):
			print("Error occured!")
			self.ser.flush()
			return
		temp_packet = PySMComPacket(packet)
		self.data_queue.put(temp_packet)

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
		if(self.mutex.acquire(blocking=True,timeout=self.mutex_timeout)):
			avlb = self.ser.inWaiting()
			self.mutex.release()
		else:
			print("Got no mutex avlb!")
		return avlb

	def __read__(self, length):
		"""
			overloaded __read__ function, inherited from SMCom
		"""
		buffer,temp,pair = [], [], SMComPy.SMCom_Pair()
		if(self.mutex.acquire(blocking=True,timeout=self.mutex_timeout)):
			temp = self.ser.read(length)
			self.mutex.release()

		if(len(temp) > 0):
			for i in temp:
				buffer.append(i)
			pair.vec = buffer
			pair.status = SMComPy.SMCOM_STATUS_SUCCESS
			return pair
		else:
			return None

	def __get_message(self,timeout):
		"""
			Instead of handling exception of queue, this function try to take data, otherwise returns None
		"""
		try:
			packet = self.data_queue.get(timeout=timeout)
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
			write_ret = self.__assign_new_id(mac, id)
			if(write_ret !=  SMComPy.SMCOM_STATUS_SUCCESS):
				print("USB port write problem")
				return False

			write_ret = self.write(user_defined_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value, [0,0,0,0,0], 5)
			if(write_ret !=  SMComPy.SMCOM_STATUS_SUCCESS):
				print("USB port write problem")
				return False

			try:
				packet = self.data_queue.get(timeout=3)
				data = packet.data
				inc_mac = tuple(data[:-3])
				inc_version = data[6:]
				if(inc_mac != mac_as_byte):
					print("Wrong id assigned to device!, integrity check failed")
					continue
				self.device_map[mac] = Wired(mac,inc_version,user_defined_id)
			except queue.Empty:
				#self.assign_new_id(i, 14)
				#time.sleep(.01)
				print("Removing device ",mac)
				self.device_map.pop(mac)
				
				#print("Cannot add device %s, not in the network or connection problem"%mac)


	def get_version(self, mac, timeout = 3):
		"""
		Takes mac address of the device and returns its version in format (MAJOR.MINOR.PATCH)(Ex. 1.0.12)
		If cannot communicate via device, returns None
		"""
		id = self.device_map[mac].user_defined_id
		write_ret = self.write(id, SMCOM_WIRED_MESSAGES.GET_VERSION.value, [], 0)
		if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
			return write_ret
		packet = self.__get_message(timeout=timeout)
		if(packet == None):
			return None
		SMComPy_version = packet.data
		version = f"{SMComPy_version[2]}.{SMComPy_version[1]}.{SMComPy_version[0]}"
		return version
	
	def __get_mac_address(self, id, timeout = 3):
		"""
		Takes receiver id as argument and returns mac address of the device with given id as string in format
		(XX:XX:XX:XX:XX:XX) (15 or 255 are reserved as public id).
		Not available for users, only used for development
		"""
		write_ret = self.write(id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value, [0,0,0,0,0], 5)
		if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
			return write_ret

		packet = self.data_queue.get(timeout = timeout)

		data = packet.data
		data = tuple(data[:-3])
		return "%02X:%02X:%02X:%02X:%02X:%02X"%data

	def __assign_new_id(self, mac_address, id):
		"""
		Takes mac adress and id to be assigned to the given mac address as arguments assigns the 
		receiver id to the device which has mac address same as given and return None.
		"""
		data = []
		data.append(id)
		mac_address = mac_address.split(':')
		for i in range(len(mac_address)):
			mac_address[i] = int(mac_address[i], base = 16)
		data.extend(mac_address)
		write_ret = self.write(SMComPy.PUBLIC_ID_4BIT.value, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_SET_NEW_ID.value, data, len(data))
		return write_ret
	
	def start_batch_measurement(self, mac, acc, freq, sample_size, notify_measurement_end = True):
		if(sample_size <= 0 or sample_size >= 1000000 or (str(freq) not in sampling_frequency_dict.keys()) or (str(acc) not in acc_range_dict.keys()) ):
			#Return arg error here
			return None
		
		id = self.device_map[mac].user_defined_id
		#Check the dictionaries
		acc_index = acc_range_dict[acc]
		freq_index = sampling_frequency_dict[str(freq)]
		#Below are all in bytes except sampling size, so no need to convert we already checked it
		data = [acc_index,freq_index,*tuple(sample_size.to_bytes(4, "little")), notify_measurement_end]

		#Check write!
		self.write(id, SMCOM_WIRED_MESSAGES.START_BATCH_MEASUREMENT.value, data, len(data))

		if(notify_measurement_end):
			#calculate end amount and give also additional time
			expected_timeout = sample_size/freq + (sample_size*1)
			data_packet = self.data_queue.get(timeout = expected_timeout)
			print(data_packet.data[0])
			return (data_packet.data[0] == WIRED_MESSAGE_STATUS.SUCCESS.value)
		
		return True
		
	def read_measurement(self, mac, sample_size, coefficient = 0, timeout = 10):
		#Wait for all packets
		byte_offset = 0 # Start from the beginning
		data_len = sample_size * 6 #Convert to bytes

		id = self.device_map[mac].user_defined_id
		data = [*tuple(byte_offset.to_bytes(4, "little")),*tuple(data_len.to_bytes(4, "little"))]

		#Check write!
		self.write(id, SMCOM_WIRED_MESSAGES.GET_BATCH_MEASUREMENT_CHUNK.value, data, len(data))

		measurement_data = [[0]*sample_size,[0]*sample_size,[0]*sample_size]
		raw_measurement_data = []

		from math import ceil
		expected_packets = ceil(data_len/240)
		while(expected_packets != 0):
			packet = self.data_queue.get(timeout = 10)
			expected_packets -= 1
			raw_data = packet.data
			measurement_status = raw_data[0]
			measurement_data_len = raw_data[1]
			raw_measurement_data.extend(raw_data[2:])
				
		it = 0
		iter = 0

		coef = coefficient
		if(coef == 0):
			coef = 1 #multiply by itself

		while(it < len(raw_measurement_data)):
			one_packet = raw_measurement_data[it:it+6]
			it += 6
			measurement_data[0][iter] = int.from_bytes(one_packet[0:2],byteorder='little',signed=True)*coef
			measurement_data[1][iter] = int.from_bytes(one_packet[2:4],byteorder='little',signed=True)*coef
			measurement_data[2][iter] = int.from_bytes(one_packet[4:6],byteorder='little',signed=True)*coef
			iter += 1

		return measurement_data

	def measure(self, mac, acc, freq, sample_size, timeout=10):
		if(self.start_batch_measurement(mac, acc, freq, sample_size, notify_measurement_end=True) == True):
			coef = self.accelerometer_coefficients[acc_range_dict[acc]]
			return self.read_measurement(mac,sample_size,coefficient= coef,timeout = timeout)
		else:
			print("Measurement failed")
			return None
	
	def get_all_telemetry(self, mac, timeout = 30):
		id = self.device_map[mac].user_defined_id
		#Check write!
		self.write(id, SMCOM_WIRED_MESSAGES.GET_ALL_TELEMETRY.value, [], 0)
		#Check also queue data receiver id!, maybe not desired message
		data = self.data_queue.get(timeout = timeout).data
		status = data[0]
		temperature = int.from_bytes(data[1:3],"little",signed=False)
		calibrated_frequency = int.from_bytes(data[3:7],"little",signed=False)
		telemetries = data[7:]

		import struct
		def convert_byte_list_to_double(bl):
			return struct.unpack('d',bytes(bl))[0]

		start = 0
		def byte_list_to_double_list(bl):
			nonlocal start
			dl = [  convert_byte_list_to_double(bl[start:start+8]),
					convert_byte_list_to_double(bl[start+8:start+16]),
					convert_byte_list_to_double(bl[start+16:start+24])]
			start += 8
			return dl
		major, minor, patch = self.get_version(id).split('.')
		
		clearance = byte_list_to_double_list(telemetries)
		crest = byte_list_to_double_list(telemetries)
		grms = byte_list_to_double_list(telemetries)
		kurtosis = byte_list_to_double_list(telemetries)
		skewness = byte_list_to_double_list(telemetries)

		telems = {
			"temperature":temperature/100,
			"calibrated_frequency":calibrated_frequency,
			"clearance":clearance,
			"crest":crest,
			"grms":grms,
			"kurtosis":kurtosis,
			"skewness":skewness
		}

		if int(patch) >= 9:
			vrms = byte_list_to_double_list(telemetries)
			peak = byte_list_to_double_list(telemetries)
			sum = byte_list_to_double_list(telemetries)
			telems["vrms"] = vrms
			telems["peak"] = peak
			telems["sum"] = sum
			
		if int(patch) >= 13:
			peak_to_peak = byte_list_to_double_list(telemetries)
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
			mac = [int(ff,16) for ff in mac.split(':')]    

		print("Device mac:",mac)
		enter_message = mac
		receiver_id = self.device_map[mac]
		write_ret = self.write(receiver_id, SMCOM_WIRED_MESSAGES.ENTER_FIRMWARE_UPDATER_MODE.value, enter_message, len(enter_message))
		
		self.ser.baudrate = 1000000
		enter_mac_return = self.data_queue.get(timeout = timeout).data
		if enter_mac_return != mac:
			return SMComPy.SMCOM_STATUS_FAIL
		try:
			if(write_ret != SMComPy.SMCOM_STATUS_SUCCESS):
				return write_ret

			print("Device entered firmware updater mode")
			bootloader_id = 12 # This is predefined id for bootloader
			no_packets = len(packets) - 1

			start_message = [*tuple(bin_file_size.to_bytes(4, "little")),*tuple(no_packets.to_bytes(4, "little")), *tuple(mac)]
			write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET_START.value, start_message, len(start_message))
			
			if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
				return write_ret
			
			read_ret = self.data_queue.get(timeout = timeout).data
			start_mac_return = read_ret[1:]
			status = read_ret[0]

			if mac != start_mac_return or status != WIRED_MESSAGE_STATUS.SUCCESS.value:
				return SMComPy.SMCOM_STATUS_FAIL
			
			for packet_no in range(no_packets + 1):
				retry = 0
				wired_packet_no = packet_no + 1
				success = False
				data_packet = [*tuple(packets[packet_no]), *tuple(mac), *tuple(wired_packet_no.to_bytes(2, "little"))]
				while retry < WIRED_FIRMWARE_MAX_RETRY_FOR_ONE_PACKET:
					retry += 1
					write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET.value, data_packet, len(data_packet))
					resp_msg_data = self.data_queue.get(timeout = timeout).data
					# print(f"retry: {retry} for packet_no {wired_packet_no}")
					if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
						# print("write error:",write_ret)
						continue
					read_ret = resp_msg_data[0]
					packet_mac_return = resp_msg_data[1:7]     # resp data[0] is wired status and 1: is mac address returned wrt the msg
					ret_packet_no = resp_msg_data[7:]
					if ret_packet_no[0] != wired_packet_no:
						return SMComPy.SMCOM_STATUS_FAIL
					if mac != packet_mac_return or read_ret != WIRED_MESSAGE_STATUS.SUCCESS.value:
						# print("data error:",read_ret)
						continue

					success = True
					# print(f"packet {wired_packet_no} success")
					break

				if not success:
					return SMComPy.SMCOM_STATUS_FAIL

			#Data transmission ended successfully
					
			write_ret = self.write(bootloader_id, SMCOM_WIRED_MESSAGES.FIRMWARE_PACKET_END.value, [*tuple(mac)], len(mac))
			if write_ret != SMComPy.SMCOM_STATUS_SUCCESS:
				return write_ret

			resp_end = self.data_queue.get(timeout = timeout).data
			read_ret = resp_end[0]
			end_mac_return = resp_end[1:]
			if mac != end_mac_return or read_ret != WIRED_MESSAGE_STATUS.SUCCESS.value:
				return SMComPy.SMCOM_STATUS_FAIL
			self.ser.baudrate = 115200
			time.sleep(12)
			time.sleep(.5)
			self.device_check()
			time.sleep(1)
			
			mac_str = "%02X:%02X:%02X:%02X:%02X:%02X"%tuple(mac)
			
			receiver_id = self.device_set[mac_str].user_defined_id
			print("Firmware Updated to version:", self.get_version(receiver_id))
		except Exception as e:
			print(e)
			print("An error occurred while firmware update")
		finally:
			self.ser.baudrate = 115200

	def __find_available_id(self):
		flag = 0; #16bit data, since max device is smaller than 16 we can use it safely

		#Now set the bits if we have a device at that bit
		for x in self.device_map.values():
			flag |= (0x01 << x.user_defined_id);
		
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
			if write_ret == SMComPy.SMCOM_STATUS_PORT_BUSY:
				pass
			elif write_ret == SMComPy.SMCOM_STATUS_SUCCESS:
				pass
			else:
				return {}
			
			time.sleep(wait_time/1000)
			read_amount = self.data_queue.qsize()

			while read_amount:
				packet:PySMComPacket = self.data_queue.get()
				if packet.message_id != SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INIT.value:
					break

				incoming_data = packet.data
				mac_adr = incoming_data[:6]
				mac_adr = "%02X:%02X:%02X:%02X:%02X:%02X"%tuple(mac_adr)
				version = incoming_data[6:]
				if(verbose):
					print("Wired device '%s' found"%mac_adr)
				id = self.__find_available_id()
				write_ret = self.__assign_new_id(mac_adr, id)
				if(write_ret == SMComPy.SMCOM_STATUS_SUCCESS):
					self.device_map[mac_adr] = Wired(mac_adr,version,id)
				else:
					print("Cannot add device ",mac_adr)
				read_amount -= 1
		
		if(len(self.device_map) == 0 and verbose):            
			print("No device found in network")
			return {}

		return self.device_map
	

	def integrity_check(self):
		if debug__ : print(f"Integrity check for {len(self.device_map)} devices")
		for i in self.device_map:
			device = self.device_map[i]
			mac_adr = i.split(':')
			mac_adr = [int(k, base = 16) for k in mac_adr]
			retry = 0
			while True:
				write_ret = self.write(device.user_defined_id, SMCOM_WIRED_MESSAGES.AUTO_ADDRESSING_INTEGRITY_CHECK.value, mac_adr, len(mac_adr))
				if write_ret == SMComPy.SMCOM_STATUS_SUCCESS:
					try:
						read_ret = self.data_queue.get(timeout = 1)
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

	update_parser = sub_parsers.add_parser('update', help = 'Update firmware of the device connected to given port with given bin file')
	update_parser.set_defaults(which='update')
	update_parser.add_argument('-p','--port', type = str, help = "port address of the device (linux /dev/ttyUSBX, win32 COMX)",default='/dev/ttyUSB0')
	update_parser.add_argument('-f','--file',type = str, help = "address of the binary file containing the firmware update",default=None)
	update_parser.add_argument('-m','--mac',type = str, help = "address of the binary file containing the firmware update",default=None)

	measurement_parser = sub_parsers.add_parser('measure', help = 'Take measurements directly from command line')
	measurement_parser.set_defaults(which='measure')
	measurement_parser.add_argument('-p','--port'    ,type = str, help = "port address of the device (linux /dev/ttyUSBX, win32 COMX",default='/dev/ttyUSB0')
	measurement_parser.add_argument('-a','--acc'     ,type = str, help = "acceleration range: Possible args: 2G, 4G, 8G, 16G",default="16G")
	measurement_parser.add_argument('-f','--freq'    ,type = int, help = "sampling frequency: Possible args: 800, 1600, 3200, 6400, 12800",default=12800)
	measurement_parser.add_argument('-s','--sample'  ,type = int, help = "sampling size: Number of samples")
	measurement_parser.add_argument('-o','--output' ,type = str, help = "output file address which measurement data will be written")
	measurement_parser.add_argument('-t','--telem'   ,help = "can be set or notset. if set, telemetries will be written at the beginning of the file")
	
	if(len(argv) <= 1):
		print("No argument is provided!")
		parser.print_help()
		exit()

	args = parser.parse_args()


	if(args.which == 'update'):
		print("Not available!")
		#dev = SMWired(port = args.port)
		#dev.firmware_update(args.mac, args.file)
	elif(args.which == 'measure'):
		print("Not available!")
		


	# if argv[1] == 'update':
	#     dev = SMWired(port = data.port)
	#     dev.firmware_update(dev.mac_address, data.binfile)
	# elif argv[1] == 'measure':
	#     dev = SMWired(port = data.port)
	#     meas = dev.measure(255, data.acc, data.freq, data.smpsize)
	#     ls = []
	#     for i in range(len(meas[0])*3):
	#         ls.append(meas[i%3][i//3])
	#     terminal = False
	#     if data.fileadr == stdout:
	#         terminal = True
	#         f = stdout

	#     if not terminal:
	#         f = open(data.fileadr, "w")
	#     if data.telem:
	#         telems = dev.get_all_telemetry(0xFF)
	#         for i in telems.keys():
	#             print(f"{i} : {telems[i]}", file = f)
	#     for i in ls:
	#         print(i, file = f)
	#     if not terminal:
	#         f.close()

if __name__ == "__main__":
	parser_function()
