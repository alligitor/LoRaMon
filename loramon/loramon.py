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
import queue
import serial
from LoRaMonUIApp import LoRaMonUIApp
from LoRaMonHelperClasses import RNS
from LoRaMonHelperClasses import RNSSerial
from LoRaMonHelperClasses import KISS
from LoRaMonHelperClasses import ROM

#returned number of packets that were captured
#the return value in Linux is only 8 bits. we need to cap it
#make this a global for use, even if rnode isn't initialized
number_of_packets_received_exit_code = 0


class RNode():
    def __init__(self, serial_instance):
        self.rns_serial = serial_instance
        self.timeout     = 100

        #These are parameters recevied from the radio
        self.r_frequency = None
        self.r_bandwidth = None
        self.r_txpower   = None
        self.r_sf        = None
        self.r_state     = None
        self.r_lock      = None
        self.r_stat_rssi = 0
        self.r_stat_snr  = 0
        self.r_battery   = None
        self.r_cr        = None
        self.r_promiscuous = None
        self.r_implicit_length = 0

        self.rssi_offset = 157

        # command line arguments
        self.sf = None
        self.cr = None
        self.txpower = None
        self.frequency = None
        self.bandwidth = None
        self.implicit_length = 0
        self.promiscuous = False

        self.detected = None

        self.eeprom = None
        self.fw_major_version = None
        self.fw_minor_version = None
        self.fw_version = None

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
        self.print_raw_data_enabled = False

        #flag for stopping the thread
        self.thread_continue = None

        #link back to the ui app, used for updating
        self.loramon_ui_app = None

        #queue for receiving updates from the UI
        self.queue_from_ui = None

        #From RNode_Firmware
        #Leaving full code here for reference to the other file
        #Config.h:    #define BATTERY_STATE_UNKNOWN     0x00
        #Config.h:    #define BATTERY_STATE_DISCHARGING 0x01
        #Config.h:    #define BATTERY_STATE_CHARGING    0x02
        #Config.h:    #define BATTERY_STATE_CHARGED     0x03

        #string, representing the value
        # ^ Charging
        # - Charged
        # v Discharging
        # ? Unknown
        self.battery_state_desc = {
            0x01 : "v",
            0x02 : "^",
            0x03 : "-",
            0x00 : "?"
        }

    def setCapturDuration(self, seconds):
        #set the start time, first
        self.capture_start_time = time.time()
        #set seconds after setting the time, to eliminate race issues with the thread
        self.duration_to_capture_for = seconds

    def device_probe(self):
        deviced_detected = False
        iterations_to_wait = 10

        sleep(2.5)
        self.detectRequest()

        while True:
            if self.detected == True:
                RNS.log("RNode connected")
                RNS.log("Firmware version: " + str(self.fw_version))
                device_detected = True
                break
            else:
                sleep(0.1)
                iterations_to_wait -= 1
                if iterations_to_wait == 0:
                    device_detected = False
                    break
        return device_detected

    def updateIUApp(self, type, value):
        if (self.loramon_ui_app != None):
            msg = {
                "type": type,
                "value": value
                }
            self.loramon_ui_app.queue_from_radio.put(msg)

    def readFromUIApp(self):
        if self.queue_from_ui:
            if not self.queue_from_ui.empty():
                msg = self.queue_from_ui.get()
                match msg['type']:
                    case "frequency":
                        #print(f"UI is requesting frequency to change to {msg['value']}")
                        self.frequency = msg['value']
                        self.setFrequency()
                    case "bandwidth":
                        #print(f"UI is requesting bandwidth to change to {msg['value']}")
                        self.bandwidth = msg['value']
                        self.setBandwidth()
                    case "spread_factor":
                        #print(f"UI is requesting spread_factor to change to {msg['value']}")
                        self.sf = msg['value']
                        self.setSpreadingFactor()
                    case "coding_rate":
                        #print(f"UI is requesting coding_rate to change to {msg['value']}")
                        self.cr = msg['value']
                        self.setCodingRate()
                    case "print_raw_data":
                        self.setPrintRawDataBytes(msg['value'])
                    case _:
                        None

    def packetReadLoop(self):
        global number_of_packets_received_exit_code

        try:
            in_frame = False
            escape = False
            command = KISS.CMD_UNKNOWN
            data_buffer = b""
            command_buffer = b""
            last_read_ms = int(time.time()*1000)
            packet_string = ""

            while self.rns_serial.IsOpen() and self.thread_continue == True:

                # process messages from the UI
                self.readFromUIApp()

                if (self.duration_to_capture_for > 0): #handle case where a capture duration is specified
                    seconds_elapsed = int(time.time() - self.capture_start_time)
                    if(seconds_elapsed > self.duration_to_capture_for):
                        return (self.number_of_packets_received)

                if self.rns_serial.InWaiting():
                    byte = ord(self.rns_serial.Read(1))
                    last_read_ms = int(time.time()*1000)

                    if (in_frame == True):
                        if (self.print_raw_data_enabled == True): #logic to print raw frame data when in frame
                            if (byte == KISS.FEND):
                                # we have detected end of a frame
                                packet_string += str(f"{byte:#0{4}x}")
                                packet_string += "<--"
                                RNS.log(packet_string)
                                packet_string = ""
                            else:
                                # we are still capturing bytes while a frame was detected
                                packet_string += f"{byte:#0{4}x} "

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
                                        self.updateIUApp("r_captured_packets", self.number_of_packets_received)
                                    case KISS.CMD_ROM_READ:
                                        self.eeprom = data_buffer
                                    case KISS.CMD_FREQUENCY:
                                        if (len(data_buffer) == 4):
                                            self.r_frequency = data_buffer[0] << 24 | data_buffer[1] << 16 | data_buffer[2] << 8 | data_buffer[3]
                                            RNS.log("Radio reporting frequency is "+str(self.r_frequency/1000000.0)+" MHz")
                                            self.updateIUApp("r_frequency", self.r_frequency)
                                            self.updateBitrate()
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_FREQUENCY")
                                    case KISS.CMD_BANDWIDTH:
                                        if (len(data_buffer) == 4):
                                            self.r_bandwidth = data_buffer[0] << 24 | data_buffer[1] << 16 | data_buffer[2] << 8 | data_buffer[3]
                                            RNS.log("Radio reporting bandwidth is "+str(self.r_bandwidth/1000.0)+" KHz")
                                            self.updateIUApp("r_bandwidth", self.r_bandwidth)
                                            self.updateBitrate()
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_BANDWIDTH")
                                    case KISS.CMD_FW_VERSION:
                                        if (len(data_buffer) == 2):
                                            self.fw_major_version = data_buffer[0]
                                            self.fw_minor_version = data_buffer[1]
                                            self.updateVersion()
                                            self.updateIUApp("fw_version", self.fw_version)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_FW_VERSION")
                                    case KISS.CMD_TXPOWER:
                                        if (len(data_buffer) == 1):
                                            self.r_txpower = data_buffer[0]
                                            RNS.log("Radio reporting TX power is "+str(self.r_txpower)+" dBm")
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_TXPOWER")
                                    case KISS.CMD_SF:
                                        if (len(data_buffer) == 1):
                                            self.r_sf = data_buffer[0]
                                            RNS.log("Radio reporting spreading factor is "+str(self.r_sf))
                                            self.updateIUApp("r_spread_factor", self.r_sf)
                                            self.updateBitrate()
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_SF")
                                    case KISS.CMD_CR:
                                        if (len(data_buffer) == 1):
                                            self.r_cr = data_buffer[0]
                                            RNS.log("Radio reporting coding rate is "+str(self.r_cr))
                                            self.updateIUApp("r_coding_rate", self.r_cr)
                                            self.updateBitrate()
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_CR")
                                    case KISS.CMD_IMPLICIT:
                                        if (len(data_buffer) == 1):
                                            self.r_implicit_length = data_buffer[0]
                                            if self.r_implicit_length != 0:
                                                RNS.log("Radio in implicit header mode, listening for packets with a length of "+str(self.r_implicit_length)+" bytes")
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_IMPLICIT")
                                    case KISS.CMD_RADIO_STATE:
                                        if (len(data_buffer) == 1):
                                            self.r_state = data_buffer[0]
                                            self.updateIUApp("r_state", self.r_state)
                                            RNS.log("Radio reporting radio state is "+str(self.r_state))
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_RADIO_STATE")
                                    case KISS.CMD_RADIO_LOCK:
                                        if (len(data_buffer) == 1):
                                            self.r_lock = data_buffer[0]
                                            self.updateIUApp("r_lock", self.r_lock)
                                            RNS.log("Radio reporting radio lock is "+str(self.r_lock))
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_RADIO_LOCK")
                                    case KISS.CMD_ERROR:
                                        if (len(data_buffer) == 1):
                                            if (data_buffer[0] == KISS.ERROR_INITRADIO):
                                                RNS.log(str(self)+" hardware initialisation error (code "+RNS.hexrep(data_buffer[0])+")")
                                            elif (data_buffer[0] == KISS.ERROR_INITRADIO):
                                                RNS.log(str(self)+" hardware TX error (code "+RNS.hexrep(data_buffer[0])+")")
                                            else:
                                                RNS.log(str(self)+" hardware error (code "+RNS.hexrep(data_buffer[0])+")")
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_ERROR")
                                    case KISS.CMD_DETECT:
                                        if (len(data_buffer) == 1):
                                            if data_buffer[0] == KISS.DETECT_RESP:
                                                self.detected = True
                                            else:
                                                self.detected = False
                                            self.updateIUApp("r_detected", self.detected)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_DETECT")
                                    case KISS.CMD_STAT_RSSI:
                                        if (len(data_buffer) == 1):
                                            self.r_stat_rssi = data_buffer[0] - self.rssi_offset
                                            RNS.log("Radio reporting rssi is "+str(self.r_stat_rssi))
                                            self.updateIUApp("r_stat_rssi", self.r_stat_rssi)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_STAT_RSSI")
                                    case KISS.CMD_STAT_SNR:
                                        if (len(data_buffer) == 1):
                                            self.r_stat_snr = int.from_bytes(bytes([data_buffer[0]]), byteorder="big", signed=True) * 0.25
                                            RNS.log("Radio reporting snr is "+str(self.r_stat_snr))
                                            self.updateIUApp("r_stat_snr", self.r_stat_snr)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_STAT_SNR")
                                    case KISS.CMD_STAT_BAT:
                                        if (len(data_buffer) == 2):
                                            RNS.log(f"Radio reporting battery state is {data_buffer[0]}, % {data_buffer[1]}")
                                            try:
                                                state = self.battery_state_desc[data_buffer[0]]
                                            except:
                                                state = "?"
                                            self.r_battery = f"S: {state}, % {data_buffer[1]}"
                                            self.updateIUApp("r_battery", self.r_battery)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_STAT_BAT")
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
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_STAT_CHTM")
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
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_STAT_PHYPRM")
                                    case KISS.CMD_PROMISC:
                                        if (len(data_buffer) == 1):
                                            RNS.log("Radio reporting promiscuous mode is " + str(data_buffer[0]))
                                            if str(data_buffer[0]) == 0:
                                                self.r_promiscuous = False
                                            else:
                                                self.r_promiscuous = True
                                            self.updateIUApp("r_promiscuous", self.r_promiscuous)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_PROMISC")
                                    case KISS.CMD_LOG:
                                        if (len(data_buffer) >= 3) and (len(data_buffer) <= 258):
                                            log_string = ""
                                            for i in range(0,data_buffer[2]+1):
                                                log_string += chr(data_buffer[3+i])
                                            RNS.log("Radio reporting log message: " +
                                                    f"Level: {data_buffer[0]}" +
                                                    f", Tag: {data_buffer[1]}" +
                                                    ", log: " + log_string)
                                        else:
                                            RNS.log(f"Wrong number of bytes {len(data_buffer)} for CMD_LOG")
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
                            if (self.print_raw_data_enabled == True):
                                packet_string += "-->"
                                packet_string += f"{byte:#0{4}x} "
                        else:
                            #we are out of frame and received a non FEND byte!!!
                            #shouldn't happen
                            packet_string += f"{byte:#0{4}x}"
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
        minstr = str(self.fw_minor_version)
        if len(minstr) == 1:
            minstr = "0"+minstr
        self.fw_version = str(self.fw_major_version)+"."+minstr

    def detectRequest(self):
        kiss_command = bytes([KISS.FEND, KISS.CMD_DETECT, KISS.DETECT_REQ, KISS.FEND, KISS.CMD_FW_VERSION, 0x00, KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
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
        self.setPromiscuousMode()
        sleep(0.5)

    def setFrequency(self):
        c1 = self.frequency >> 24
        c2 = self.frequency >> 16 & 0xFF
        c3 = self.frequency >> 8 & 0xFF
        c4 = self.frequency & 0xFF
        data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_FREQUENCY])+data+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring frequency for "+self(str))

    def setBandwidth(self):
        c1 = self.bandwidth >> 24
        c2 = self.bandwidth >> 16 & 0xFF
        c3 = self.bandwidth >> 8 & 0xFF
        c4 = self.bandwidth & 0xFF
        data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_BANDWIDTH])+data+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring bandwidth for "+self(str))

    def setTXPower(self):
        txp = bytes([self.txpower])

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_TXPOWER])+txp+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring TX power for "+self(str))

    def setSpreadingFactor(self):
        sf = bytes([self.sf])

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_SF])+sf+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring spreading factor for "+self(str))

    def setCodingRate(self):
        cr = bytes([self.cr])

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_CR])+cr+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring coding rate for "+self(str))

    def setImplicitLength(self):
        if self.implicit_length != 0:
            length = KISS.escape(bytes([self.implicit_length]))

            kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_IMPLICIT])+length+bytes([KISS.FEND])
            written = self.rns_serial.Write(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while configuring implicit header mode for "+self(str))

    def setRadioState(self, state):
        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_RADIO_STATE])+bytes([state])+bytes([KISS.FEND])
        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring radio state for "+self(str))

    def setPromiscuousMode(self):
        if self.promiscuous == True:
            kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x01, KISS.FEND])
        else:
            kiss_command = bytes([KISS.FEND, KISS.CMD_PROMISC, 0x00, KISS.FEND])

        written = self.rns_serial.Write(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring promiscuous mode for "+self(str))

    def setPrintRawDataBytes(self, state):
        if state == True:
            self.print_raw_data_enabled = True
            # for debugging
            # RNS.log("setPrintRawDataBytes Received True")
        else:
            self.print_raw_data_enabled = False
            # for debugging
            # RNS.log("setPrintRawDataBytes Received False")

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
        parser.add_argument("-Q", action="store_true", help="Quiet mode, no logging")
        parser.add_argument("-R", action="store_true", help="Raw frame mode")
        parser.add_argument("-P", action="store_true", help="Set promiscuous mode")
        parser.add_argument("-U", action="store_true", help="Use a URWID based UI")

        parser.add_argument("port", nargs="?", default=None, help="Serial port where RNode is attached", type=str)
        args = parser.parse_args()

        console_output = False
        write_to_disk = False
        write_dir = None

        if args.duration and args.U:
            print("UI mode and timed capture are not compatible")
            exit(number_of_packets_received_exit_code)

        if args.Q and args.U:
            print("Quite mode and timed capture are not compatible")
            exit(number_of_packets_received_exit_code)

        if args.port:
            if not args.Q:
                RNS.log("Opening serial port "+args.port+"...")
            rnode = None
            serial_device = None
            rnode_baudrate = 115200

            try:
                serial_device = serial.Serial(
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
        else:
            print("")
            parser.print_help()
            print("")
            exit(number_of_packets_received_exit_code)

        # create an RNS Serial device that uses locking for read / write
        rns_serial = RNSSerial(serial_device)

        # create the RNode object and give it the lockable serial device
        rnode = RNode(rns_serial)

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

        if args.freq:
            rnode.frequency = args.freq
        else:
            RNS.log("Freq is a required parameter.")
            exit(number_of_packets_received_exit_code)

        if args.bw:
            rnode.bandwidth = args.bw
        else:
            RNS.log("BW is a required parameter.")
            exit(number_of_packets_received_exit_code)

        if args.txp and (args.txp >= 0 and args.txp <= 17):
            rnode.txpower = args.txp
        else:
            rnode.txpower = 0

        if args.sf:
            rnode.sf = args.sf
        else:
            RNS.log("SF is a required parameter.")
            exit(number_of_packets_received_exit_code)

        if args.cr:
            rnode.cr = args.cr
        else:
            RNS.log("CR is a required parameter.")
            exit(number_of_packets_received_exit_code)

        if args.implicit:
            rnode.implicit_length = args.implicit
        else:
            rnode.implicit_length = 0

        if args.hex:
            rnode.print_hex = True
        else:
            rnode.print_hex = False

        if args.P:
            rnode.promiscuous = True
        else:
            rnode.promiscuous = False

        if (args.R):
            rnode.setPrintRawDataBytes(True)

        if not args.W and not args.console:
            RNS.log("Warning! No output destination specified! You won't see any captured packets.")

        #turn off all logging until we probe the RNode device
        #we don't want leftover packets, data, etc. to spew out
        RNS.log_enabled = False

        #Get the background thread going that will read from radio and help detect it
        rnode.callback = packet_captured
        rnode.console_output = console_output
        rnode.write_to_disk = write_to_disk
        rnode.write_dir = write_dir

        #set the flag that keeps the thread running
        rnode.thread_continue = True
        thread = threading.Thread(target=rnode.packetReadLoop)
        thread.daemon = True
        thread.start()

        #kick off a probe
        if rnode.device_probe() == False:
            print("Serial port opened, but RNode did not respond.")
            rnode.thread_continue = False
        else:
            #decide whether we are running on console, or URWID based UI
            if (args.U):
                #enable logging again, now that radio is detected
                #don't look at args.Q, it's not compatable with UI mode
                RNS.log_enabled = True

                #UI Mode
                RNS.queue_to_ui = queue.Queue()
                rnode.queue_from_ui = queue.Queue()

                loramon_ui_app = LoRaMonUIApp(RNS.queue_to_ui, rnode.queue_from_ui)

                rnode.loramon_ui_app = loramon_ui_app

                #initialize the radio
                rnode.initRadio()

                #set the setting for print raw data
                rnode.updateIUApp("print_raw_data", args.R)

                #set the fw version on the ui
                rnode.updateIUApp("r_fw_version", str(rnode.fw_version))

                loramon_ui_app.run()
                rnode.thread_continue = False
                None
            else:
                #Console mode

                if args.Q:
                    RNS.log_enabled = False
                else:
                    RNS.log_enabled = True

                #initialize the radio
                rnode.initRadio()

                # set the duration here, after radio has been initialized
                # duration mode is only available in console mode
                rnode.setCapturDuration(args.duration)
                RNS.log(f"Capture Duration {rnode.duration_to_capture_for}")

        #program is ending, by the it gets here
        #wait for the thread to finish
        thread.join()
        exit(number_of_packets_received_exit_code)
    except KeyboardInterrupt:
        print("")
        exit(number_of_packets_received_exit_code)

if __name__ == "__main__":
    main()
