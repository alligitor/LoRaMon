#!/usr/bin/python3
from time import sleep
import argparse
import threading
import ctypes
import os
import struct
import datetime
import time
import math
import traceback
from importlib import util

class RNS():
	log_enabled = True

	@staticmethod
	def log(msg):
		if RNS.log_enabled:
			logtimefmt   = "%Y-%m-%d %H:%M:%S"
			timestamp = time.time()
			logstring = "["+time.strftime(logtimefmt)+"] "+msg
			print(logstring)

	@staticmethod
	def hexrep(data, delimit=True):
		delimiter = ":"
		if not delimit:
			delimiter = ""
		hexrep = delimiter.join("{:02x}".format(c) for c in data)
		return hexrep

	@staticmethod
	def prettyhexrep(data):
		delimiter = ""
		hexrep = "<"+delimiter.join("{:02x}".format(c) for c in data)+">"
		return hexrep

class KISS():
	FEND			= 0xC0
	FESC			= 0xDB
	TFEND			= 0xDC
	TFESC			= 0xDD

	CMD_UNKNOWN		= 0xFE
	CMD_DATA		= 0x00
	CMD_FREQUENCY	= 0x01
	CMD_BANDWIDTH	= 0x02
	CMD_TXPOWER		= 0x03
	CMD_SF			= 0x04
	CMD_CR          = 0x05
	CMD_RADIO_STATE = 0x06
	CMD_RADIO_LOCK	= 0x07
	CMD_DETECT		= 0x08
	CMD_IMPLICIT    = 0x09
	CMD_PROMISC     = 0x0E
	CMD_READY       = 0x0F
	CMD_STAT_RX		= 0x21
	CMD_STAT_TX		= 0x22
	CMD_STAT_RSSI	= 0x23
	CMD_STAT_SNR	= 0x24
	CMD_STAT_CHTM   = 0x25
	CMD_STAT_PHYPRM = 0x26
	CMD_STAT_BAT    = 0x27
	CMD_STAT_CSMA   = 0x28
	CMD_BLINK		= 0x30
	CMD_RANDOM		= 0x40
	CMD_FW_VERSION  = 0x50
	CMD_ROM_READ    = 0x51
	CMD_ROM_WRITE   = 0x52
	CMD_CONF_SAVE   = 0x53
	CMD_CONF_DELETE = 0x54
	DETECT_REQ      = 0x73
	DETECT_RESP     = 0x46
	RADIO_STATE_OFF = 0x00
	RADIO_STATE_ON	= 0x01
	RADIO_STATE_ASK = 0xFF

	CMD_ERROR		    = 0x90
	ERROR_INITRADIO     = 0x01
	ERROR_TXFAILED	    = 0x02
	ERROR_EEPROM_LOCKED	= 0x03

	@staticmethod
	def escape(data):
		data = data.replace(bytes([0xdb]), bytes([0xdb, 0xdd]))
		data = data.replace(bytes([0xc0]), bytes([0xdb, 0xdc]))
		return data

class ROM():
	PRODUCT_RNODE  = chr(0x03)
	MODEL_A4       = chr(0xA4)
	MODEL_A9       = chr(0xA9)

	ADDR_PRODUCT   = chr(0x00)
	ADDR_MODEL     = chr(0x01)
	ADDR_HW_REV    = chr(0x02)
	ADDR_SERIAL    = chr(0x03)
	ADDR_MADE	   = chr(0x07)
	ADDR_CHKSUM	   = chr(0x0B)
	ADDR_SIGNATURE = chr(0x1B)
	ADDR_INFO_LOCK = chr(0x9B)
	ADDR_CONF_SF   = chr(0x9C)
	ADDR_CONF_CR   = chr(0x9D)
	ADDR_CONF_TXP  = chr(0x9E)
	ADDR_CONF_BW   = chr(0x9F)
	ADDR_CONF_FREQ = chr(0xA3)
	ADDR_CONF_OK   = chr(0xA7)

	INFO_LOCK_BYTE = chr(0x73)
	CONF_OK_BYTE   = chr(0x73)

class RNode():
	def __init__(self, serial_instance):
		self.serial = serial_instance
		self.timeout     = 100

		self.r_frequency = None
		self.r_bandwidth = None
		self.r_txpower   = None
		self.r_sf        = None
		self.r_state     = None
		self.r_lock      = None
		self.r_stat_rssi = 0
		self.r_stat_snr  = 0
		self.r_implicit_length = 0

		self.rssi_offset = 157

		self.sf = None
		self.cr = None
		self.txpower = None
		self.frequency = None
		self.bandwidth = None
		self.implicit_length = 0

		self.detected = None

		self.eeprom = None
		self.major_version = None
		self.minor_version = None
		self.version = None

		self.provisioned = None
		self.product = None
		self.model = None
		self.hw_rev = None
		self.made = None
		self.serialno = None
		self.checksum = None
		self.signature = None
		self.signature_valid = False
		self.vendor = None

		self.min_freq = None
		self.max_freq = None
		self.max_output = None

		self.configured = None
		self.conf_sf = None
		self.conf_cr = None
		self.conf_txpower = None
		self.conf_frequency = None
		self.conf_bandwidth = None

		#channel parameters
		self.ats = None  #Air time, short term
		self.atl = None  #air time, long term
		self.cls = None  #Channel utilization, short term
		self.cll = None  #Channel utilization, long term
		self.crs = None  #Current RSSI
		self.nfl = None  #Current noise floor
		self.ntf = None  #Interference

		#phy parameters
		self.lst = None  #Symbol time
		self.lsr = None  #Symbol rate
		self.prs = None  #Preamble length
		self.prt = None  #PRT
		self.cst = None  #CSMA Slot MS
		self.dft = None  #DIFS MS

		self.print_hex = None

		#duration of time, in seconds, to capture
		self.duration_to_capture_for = 0
		#total number of packets captured
		self.number_of_packets_received = 0
		self.capture_start_time = time.time()

		#flag for printing raw bytes from RNode
		self.raw_data_enabled = False

	def setCapturDuration(self, seconds):
		#set the start time, first
		self.capture_start_time = time.time()
		#set seconds after setting the time, to eliminate race issues with the thread
		self.duration_to_capture_for = seconds

	def packetReadLoop(self):
		try:
			in_frame = False
			escape = False
			command = KISS.CMD_UNKNOWN
			data_buffer = b""
			command_buffer = b""
			last_read_ms = int(time.time()*1000)

			while self.serial.is_open:
				if (self.duration_to_capture_for > 0): #handle case where a capture duration is specified
					seconds_elapsed = int(time.time() - self.capture_start_time)
					if(seconds_elapsed > self.duration_to_capture_for):
						return (self.number_of_packets_received)

				if self.serial.in_waiting:
					byte = ord(self.serial.read(1))
					last_read_ms = int(time.time()*1000)

					if (in_frame == True):
						if (self.raw_data_enabled == True): #logic to print raw frame data when in frame
							if (byte == KISS.FEND):
								# we have detected end of a frame
								print(f"{byte:#0{4}x}", end="")
								print("<--")
							else:
								# we are still capturing bytes while a frame was detected
								print(f"{byte:#0{4}x} ", end="")

						if (byte == KISS.FEND): #received FEND, which signals end of a frame
							#first make sure we actually received a command
							if (len(command_buffer) == 0): #double FEND
								#detected a FEND FEND situation
								#stay in frame
								None
							else: #we have a command, handle it
								match command_buffer[0]:
									case KISS.CMD_DATA:
										self.processIncoming(data_buffer)
										self.number_of_packets_received += 1
										if (self.number_of_packets_received > 255):
											number_of_packets_received_exit_code = 255
										else:
											number_of_packets_received_exit_code = self.number_of_packets_received
									case KISS.CMD_ROM_READ:
										self.eeprom = data_buffer
									case KISS.CMD_FREQUENCY:
										if (len(data_buffer) == 4):
											self.r_frequency = data_buffer[0] << 24 | data_buffer[1] << 16 | data_buffer[2] << 8 | data_buffer[3]
											RNS.log("Radio reporting frequency is "+str(self.r_frequency/1000000.0)+" MHz")
											self.updateBitrate()
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for frequency")
									case KISS.CMD_BANDWIDTH:
										if (len(data_buffer) == 4):
											self.r_bandwidth = data_buffer[0] << 24 | data_buffer[1] << 16 | data_buffer[2] << 8 | data_buffer[3]
											RNS.log("Radio reporting bandwidth is "+str(self.r_bandwidth/1000.0)+" KHz")
											self.updateBitrate()
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for bandwith")
									case KISS.CMD_FW_VERSION:
										if (len(data_buffer) == 2):
											self.major_version = data_buffer[0]
											self.minor_version = data_buffer[1]
											self.updateVersion()
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for version")
									case KISS.CMD_TXPOWER:
										if (len(data_buffer) == 1):
											self.r_txpower = data_buffer[0]
											RNS.log("Radio reporting TX power is "+str(self.r_txpower)+" dBm")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for tx power")
									case KISS.CMD_SF:
										if (len(data_buffer) == 1):
											self.r_sf = data_buffer[0]
											RNS.log("Radio reporting spreading factor is "+str(self.r_sf))
											self.updateBitrate()
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for spreading factor")
									case KISS.CMD_CR:
										if (len(data_buffer) == 1):
											self.r_cr = data_buffer[0]
											RNS.log("Radio reporting coding rate is "+str(self.r_cr))
											self.updateBitrate()
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for coding rate")
									case KISS.CMD_IMPLICIT:
										if (len(data_buffer) == 1):
											self.r_implicit_length = data_buffer[0]
											if self.r_implicit_length != 0:
												RNS.log("Radio in implicit header mode, listening for packets with a length of "+str(self.r_implicit_length)+" bytes")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for implicit header")
									case KISS.CMD_RADIO_STATE:
										if (len(data_buffer) == 1):
											self.r_state = data_buffer[0]
											RNS.log("Radio reporting radio state is "+str(self.r_state))
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for radio state")
									case KISS.CMD_RADIO_LOCK:
										if (len(data_buffer) == 1):
											self.r_lock = data_buffer[0]
											RNS.log("Radio reporting radio lock is "+str(self.r_lock))
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for radio lock")
									case KISS.CMD_ERROR:
										if (len(data_buffer) == 1):
											if (data_buffer[0] == KISS.ERROR_INITRADIO):
												RNS.log(str(self)+" hardware initialisation error (code "+RNS.hexrep(data_buffer[0])+")")
											elif (data_buffer[0] == KISS.ERROR_INITRADIO):
												RNS.log(str(self)+" hardware TX error (code "+RNS.hexrep(data_buffer[0])+")")
											else:
												RNS.log(str(self)+" hardware error (code "+RNS.hexrep(data_buffer[0])+")")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for error")
									case KISS.CMD_DETECT:
										if (len(data_buffer) == 1):
											if data_buffer[0] == KISS.DETECT_RESP:
												self.detected = True
											else:
												self.detected = False
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for detect")
									case KISS.CMD_STAT_RSSI:
										if (len(data_buffer) == 1):
											self.r_stat_rssi = data_buffer[0] - self.rssi_offset
											RNS.log("Radio reporting rssi is "+str(self.r_stat_rssi))
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for rssi")
									case KISS.CMD_STAT_SNR:
										if (len(data_buffer) == 1):
											self.r_stat_snr = int.from_bytes(bytes([data_buffer[0]]), byteorder="big", signed=True) * 0.25
											RNS.log("Radio reporting snr is "+str(self.r_stat_snr))
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for snr")
									case KISS.CMD_STAT_BAT:
										if (len(data_buffer) == 2):
											RNS.log(f"Radio reporting battery state is {data_buffer[0]}, % {data_buffer[1]}")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for battery")
									case KISS.CMD_STAT_CHTM:
										if (len(data_buffer) == 11):
											self.ats = data_buffer[0] << 8 | data_buffer[1]
											self.atl = data_buffer[2] << 8 | data_buffer[3]
											self.cls = data_buffer[4] << 8 | data_buffer[5]
											self.cll = data_buffer[6] << 8 | data_buffer[7]
											self.crs = data_buffer[8]					
											self.nfl = data_buffer[9]					
											self.ntf = data_buffer[10]					
											RNS.log(f"Radio reporting CHTM: " +
													f"ats:{self.ats}, " +
													f"atl:{self.atl}, " +
													f"cls:{self.cls}, " +
													f"cll:{self.cll}, " +
													f"crs:{self.crs}, " +
													f"nfl:{self.nfl}, " +
													f"ntf:{self.ntf}")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for chtm")
									case KISS.CMD_STAT_PHYPRM:
										if (len(data_buffer) == 12):
											self.lst = data_buffer[0] << 8 | data_buffer[1]
											self.lsr = data_buffer[2] << 8 | data_buffer[3]
											self.prs = data_buffer[4] << 8 | data_buffer[5]
											self.prt = data_buffer[6] << 8 | data_buffer[7]
											self.cst = data_buffer[8] << 8 | data_buffer[9]
											self.dft = data_buffer[10] << 8 | data_buffer[11]
											RNS.log(f"Radio reporting PHYPRM: " +
													f"lst:{self.lst}, " +
													f"lsr:{self.lsr}, " +
													f"prs:{self.prs}, " +
													f"prt:{self.prt}, " +
													f"cst:{self.cst}, " +
													f"dft:{self.dft}")
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for phyprm")
									case KISS.CMD_PROMISC:
										if (len(data_buffer) == 1):
											RNS.log("Radio reporting promiscuous mode is " + str(data_buffer[0]))
										else:
											RNS.log(f"Wrong number of bytes {len(data_buffer)} for promisc")
									case _: #any command not handled here
										RNS.log(f"Received unhandled command {command_buffer[0]}")

								# set all buffer to empty
								in_frame = False
								data_buffer = b""
								command_buffer = b""
						else: #still in frame, accumulate received bytes in data_buffer
							if (len(command_buffer) == 0): #first byte after FEND is the command
								command_buffer = bytes([byte])
							#add the received byte to data_buffer but account for escape characters
							else: #receive bytes after the command
								if (byte == KISS.FESC):
									escape = True
								else:
									if (escape):
										if (byte == KISS.TFEND):
											byte = KISS.FEND
										elif (byte == KISS.TFESC):
											byte = KISS.FESC
										else:
											RNS.log(f"Recieved bad escape code {byte}")
										escape = False
									data_buffer = data_buffer+bytes([byte])
					else: #in_frame != True
						if (byte == KISS.FEND):
							#not in frame and received FEND.  start a new frame
							in_frame = True
							command = KISS.CMD_UNKNOWN
							data_buffer = b""
							command_buffer = b""
							if (self.raw_data_enabled == True):
								print() #print a new line, in case there were out of frame characters
								print("-->", end="")
								print(f"{byte:#0{4}x} ", end="")
						else:
							#we are out of frame and received a non FEND byte!!!
							#shouldn't happen
							print(f"{byte:#0{4}x}", end="")
							None
				else:
					time_since_last = int(time.time()*1000) - last_read_ms
					if in_frame == True and time_since_last > self.timeout:
						RNS.log(str(self)+" serial read timeout while receiving frame")
						data_buffer = b""
						in_frame = False
						command = KISS.CMD_UNKNOWN
						escape = False
					sleep(0.08)

		except Exception as e:
			RNS.log("Error while reading from serial port")
			traceback.print_exc()
			return(0)

	def processIncoming(self, data):
		self.callback(data, self)

	def updateBitrate(self):
		try:
			self.bitrate = self.sf * ( (4.0/self.cr) / (math.pow(2,self.sf)/(self.bandwidth/1000)) ) * 1000
			self.bitrate_kbps = round(self.bitrate/1000.0, 2)
		except:
			self.bitrate = 0

	def updateVersion(self):
		minstr = str(self.minor_version)
		if len(minstr) == 1:
			minstr = "0"+minstr
		self.version = str(self.major_version)+"."+minstr

	def detect(self):
		kiss_command = bytes([KISS.FEND, KISS.CMD_DETECT, KISS.DETECT_REQ, KISS.FEND, KISS.CMD_FW_VERSION, 0x00, KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring spreading factor for "+self(str))

	def initRadio(self):
		self.setFrequency()
		self.setBandwidth()
		self.setTXPower()
		self.setSpreadingFactor()
		self.setCodingRate()
		self.setImplicitLength()
		self.setRadioState(KISS.RADIO_STATE_ON)

	def setFrequency(self):
		c1 = self.frequency >> 24
		c2 = self.frequency >> 16 & 0xFF
		c3 = self.frequency >> 8 & 0xFF
		c4 = self.frequency & 0xFF
		data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_FREQUENCY])+data+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring frequency for "+self(str))

	def setBandwidth(self):
		c1 = self.bandwidth >> 24
		c2 = self.bandwidth >> 16 & 0xFF
		c3 = self.bandwidth >> 8 & 0xFF
		c4 = self.bandwidth & 0xFF
		data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_BANDWIDTH])+data+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring bandwidth for "+self(str))

	def setTXPower(self):
		txp = bytes([self.txpower])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_TXPOWER])+txp+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring TX power for "+self(str))

	def setSpreadingFactor(self):
		sf = bytes([self.sf])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_SF])+sf+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring spreading factor for "+self(str))

	def setCodingRate(self):
		cr = bytes([self.cr])

		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_CR])+cr+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring coding rate for "+self(str))

	def setImplicitLength(self):
		if self.implicit_length != 0:
			length = KISS.escape(bytes([self.implicit_length]))

			kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_IMPLICIT])+length+bytes([KISS.FEND])
			written = self.serial.write(kiss_command)
			if written != len(kiss_command):
				raise IOError("An IO error occurred while configuring implicit header mode for "+self(str))

	def setRadioState(self, state):
		kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_RADIO_STATE])+bytes([state])+bytes([KISS.FEND])
		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring radio state for "+self(str))

	def setPromiscuousMode(self, state):
		if state == True:
			kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x01, KISS.FEND])
		else:
			kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x00, KISS.FEND])

		written = self.serial.write(kiss_command)
		if written != len(kiss_command):
			raise IOError("An IO error occurred while configuring promiscuous mode for "+self(str))

#returned number of packets that were captured
#the return value in Linux is only 8 bits. we need to cap it
#make this a global for use, even if rnode isn't initialized
number_of_packets_received_exit_code = 0

def device_probe(rnode):
	sleep(2.5)
	rnode.detect()
	sleep(0.1)
	if rnode.detected == True:
		RNS.log("RNode connected")
		RNS.log("Firmware version: " + str(rnode.version))
		return True
	else:
		raise IOError("Got invalid response while detecting device")

def packet_captured(data, rnode_instance):
	if rnode_instance.console_output:
		if rnode_instance.print_hex:
			if len(data) == 1:
				data = [data]
			datastring = "\n"+RNS.hexrep(data)+"\n"
		else:
			datastring = str(data)

		RNS.log("["+str(rnode_instance.r_stat_rssi)+" dBm] [SNR "+str(rnode_instance.r_stat_snr)+" dB] ["+str(len(data))+" bytes]\t"+datastring);
	if rnode_instance.write_to_disk:
		try:
			filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S.%f")+".pkt"
			file = open(rnode_instance.write_dir+"/"+filename, "wb")
			file.write(data)
			file.close()
		except Exception as e:
			RNS.log("Error while writing packet to disk")
			os._exit(number_of_packets_received_exit_code)


def main():
	try:
		if not util.find_spec("serial"):
			raise ImportError("Serial module could not be found")
	except ImportError:
		print("")
		print("RNode Config Utility needs pyserial to work.")
		print("You can install it with: pip3 install pyserial")
		print("")
		exit(number_of_packets_received_exit_code)

	import serial

	try:
		parser = argparse.ArgumentParser(description="LoRa packet sniffer for RNode hardware.")
		parser.add_argument("-C", "--console", action="store_true", help="Print captured packets to the console")
		parser.add_argument("-H", "--hex", action="store_true", help="Print out packets as hexadecimal")
		parser.add_argument("-W", action="store", metavar="directory", type=str, default=None, help="Write captured packets to a directory")
		parser.add_argument("--freq", action="store", metavar="Hz", type=int, default=None, help="Frequency in Hz")
		parser.add_argument("--bw", action="store", metavar="Hz", type=int, default=None, help="Bandwidth in Hz")
		parser.add_argument("--txp", action="store", metavar="dBm", type=int, default=None, help="TX power in dBm")
		parser.add_argument("--sf", action="store", metavar="factor", type=int, default=None, help="Spreading factor")
		parser.add_argument("--cr", action="store", metavar="rate", type=int, default=None, help="Coding rate")
		parser.add_argument("--implicit", action="store", metavar="length", type=int, default=None, help="Packet length in implicit header mode")
		parser.add_argument("--duration", action="store", metavar="seconds", type=int, default=0,help="Duration of time to capture packets")
		parser.add_argument("-Q", action="store_true", help="Quite mode, no logging")
		parser.add_argument("-R", action="store_true", help="Raw frame mode")
		parser.add_argument("-P", action="store_true", help="Set promiscuous mode")

		parser.add_argument("port", nargs="?", default=None, help="Serial port where RNode is attached", type=str)
		args = parser.parse_args()

		console_output = False
		write_to_disk = False
		write_dir = None

		if args.Q:
			RNS.log_enabled = False

		if args.console:
			console_output = True

		if args.W:
			if not os.path.isdir(args.W):
				try:
					os.mkdir(args.W)
					write_to_disk = True
					write_dir = args.W
				except Exception as e:
					RNS.log("Could not open or create specified directory")
			else:
				write_to_disk = True
				write_dir = args.W

		if args.port:
			RNS.log("Opening serial port "+args.port+"...")
			rnode = None
			rnode_serial = None
			rnode_baudrate = 115200

			try:
				rnode_serial = serial.Serial(
					port = args.port,
					baudrate = rnode_baudrate,
					bytesize = 8,
					parity = serial.PARITY_NONE,
					stopbits = 1,
					xonxoff = False,
					rtscts = False,
					timeout = 0,
					inter_byte_timeout = None,
					write_timeout = None,
					dsrdtr = False
				)
			except Exception as e:
				RNS.log("Could not open the specified serial port. The contained exception was:")
				RNS.log(str(e))
				exit(number_of_packets_received_exit_code)

			rnode = RNode(rnode_serial)

			if (args.R):
				rnode.raw_data_enabled = True

			rnode.callback = packet_captured
			rnode.console_output = console_output
			rnode.write_to_disk = write_to_disk
			rnode.write_dir = write_dir

			thread = threading.Thread(target=rnode.packetReadLoop)
			thread.daemon = True
			thread.start()

			try:
				device_probe(rnode)
			except Exception as e:
				RNS.log("Serial port opened, but RNode did not respond.")
				print(e)
				exit(number_of_packets_received_exit_code)

			if not (args.freq and args.bw and args.sf and args.cr):
				RNS.log("Please input startup configuration:")
				print("")

			if args.freq:
				rnode.frequency = args.freq
			else:
				print("Frequency in Hz:\t", end=' ')
				rnode.frequency = int(input())


			if args.bw:
				rnode.bandwidth = args.bw
			else:
				print("Bandwidth in Hz:\t", end=' ')
				rnode.bandwidth = int(input())

			if args.txp and (args.txp >= 0 and args.txp <= 17):
				rnode.txpower = args.txp
			else:
				rnode.txpower = 0

			if args.sf:
				rnode.sf = args.sf
			else:
				print("Spreading factor:\t", end=' ')
				rnode.sf = int(input())

			if args.cr:
				rnode.cr = args.cr
			else:
				print("Coding rate:\t\t", end=' ')
				rnode.cr = int(input())

			if args.implicit:
				rnode.implicit_length = args.implicit
			else:
				rnode.implicit_length = 0

			if args.hex:
				rnode.print_hex = True
			else:
				rnode.print_hex = False

			rnode.initRadio()

			if args.P:
				rnode.setPromiscuousMode(True)
				sleep(0.5)

			# set the duration here, after radio has been initialized
			rnode.setCapturDuration(args.duration)

			RNS.log(f"Capture Duration {rnode.duration_to_capture_for}")

			if not args.W and not args.console:
				RNS.log("Warning! No output destination specified! You won't see any captured packets.")

			#wait for the thread to finish
			thread.join()
			exit(number_of_packets_received_exit_code)
		else:
			print("")
			parser.print_help()
			print("")
			exit(number_of_packets_received_exit_code)

	except KeyboardInterrupt:
		print("")
		exit(number_of_packets_received_exit_code)

if __name__ == "__main__":
	main()
