import datetime
import time
import threading

class RNSSerial():
    def __init__(self, serial_port):
        self.serial_device = serial_port

        # Create a lock object
        self.lock = threading.Lock()

    def Write(self, data):
        count = None
        with self.lock:
            count = self.serial_device.write(data)
        return count

    def Read(self, count):
        byte = None
        with self.lock:
            byte = self.serial_device.read(count)
        return byte

    def IsOpen(self):
        state = None
        with self.lock:
            state = self.serial_device.is_open
        return state

    def InWaiting(self):
        state = None
        with self.lock:
            state = self.serial_device.in_waiting
        return state

class RNS():
    log_enabled = True
    queue_to_ui = None

    def __init__ (self):
        RNS.log_enabled = True
        RNS.queue_to_ui = None

    @staticmethod
    def log(msg):
        if RNS.log_enabled:
            logtimefmt   = "%Y-%m-%d %H:%M:%S"
            timestamp = time.time()
            logstring = "["+time.strftime(logtimefmt)+"] "+msg
            if RNS.queue_to_ui == None:
                print(logstring)
            else:
                msg_to_ui = {
                    "type": "LogMessageFromRadio",
                    "value": logstring
                    }
                RNS.queue_to_ui.put(msg_to_ui)

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
    FEND            = 0xC0
    FESC            = 0xDB
    TFEND           = 0xDC
    TFESC           = 0xDD

    CMD_UNKNOWN     = 0xFE
    CMD_DATA        = 0x00
    CMD_FREQUENCY   = 0x01
    CMD_BANDWIDTH   = 0x02
    CMD_TXPOWER     = 0x03
    CMD_SF          = 0x04
    CMD_CR          = 0x05
    CMD_RADIO_STATE = 0x06
    CMD_RADIO_LOCK  = 0x07
    CMD_DETECT      = 0x08
    CMD_IMPLICIT    = 0x09
    CMD_PROMISC     = 0x0E
    CMD_READY       = 0x0F
    CMD_STAT_RX     = 0x21
    CMD_STAT_TX     = 0x22
    CMD_STAT_RSSI   = 0x23
    CMD_STAT_SNR    = 0x24
    CMD_STAT_CHTM   = 0x25
    CMD_STAT_PHYPRM = 0x26
    CMD_STAT_BAT    = 0x27
    CMD_STAT_CSMA   = 0x28
    CMD_BLINK       = 0x30
    CMD_RANDOM      = 0x40
    CMD_FW_VERSION  = 0x50
    CMD_ROM_READ    = 0x51
    CMD_ROM_WRITE   = 0x52
    CMD_CONF_SAVE   = 0x53
    CMD_CONF_DELETE = 0x54
    CMD_LOG         = 0x80
    DETECT_REQ      = 0x73
    DETECT_RESP     = 0x46
    RADIO_STATE_OFF = 0x00
    RADIO_STATE_ON  = 0x01
    RADIO_STATE_ASK = 0xFF

    CMD_ERROR           = 0x90
    ERROR_INITRADIO     = 0x01
    ERROR_TXFAILED      = 0x02
    ERROR_EEPROM_LOCKED = 0x03

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
    ADDR_MADE       = chr(0x07)
    ADDR_CHKSUM       = chr(0x0B)
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
