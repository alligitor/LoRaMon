# LoRa packet sniffer for RNode hardware

## Intro

This utility allows you to sniff LoRa networks with an [RNode](https://unsigned.io/projects/rnode/), and dump captured packets to the console or files.

```sh
usage: loramon [-h] [-C] [-W directory] [--freq Hz] [--bw Hz] [--txp dBm]
               [--sf factor] [--cr rate]
               [--duration seconds] [-Q]
               [-R]
               [-P]
               [-U]
               [port]

LoRa packet sniffer for RNode hardware.

positional arguments:
  port           Serial port where RNode is attached

optional arguments:
  -h, --help     show this help message and exit
  -C, --console  Print captured packets to the console
  -W directory   Write captured packets to a directory
  --freq Hz      Frequency in Hz
  --bw Hz        Bandwidth in Hze
  --txp dBm      TX power in dBm
  --sf factor    Spreading factor
  --cr rate      Coding rate
  --duration s   Duration to scan for in seconds
  -Q             Quiet mode. Don't log any messages after start up
  -R             Print out raw bytes of the frame
  -P             Set promiscuous mode
  -U             Use a text based UI
```

## Installation

If you already have Python3 and pip installed, you can easily install LoRaMon through pip:

```sh
pip3 install loramon
```

On Arch Linux it is also possible to install using the `loramon` package from the [AUR](https://aur.archlinux.org/packages/loramon).

If you want to install directly from this repository, first install the dependencies:

```sh
sudo apt install python3 python3-pip
sudo pip3 install pyserial
```

And then clone the repository and make LoRaMon executable:

```sh
git clone https://github.com/markqvist/LoRaMon.git
cd LoRaMon
chmod a+x loramon
./loramon --help
```

## Usage Examples

### Dump to console

Listens on a specified frequency and displays captured packets in the console.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C
```

### Dump to console and disk

Like above, but also writes all captured packets individually to a specified directory.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C -W capturedir
```

### Sniff implicit header mode packets

If you want to sniff LoRa packets with implicit header mode, use the --implicit option along with the length in bytes of the expected packet. This mode needs an RNode with a firmware version of at least 1.17.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C -W capturedir --implicit 12
```

### Use Raw mode to print all the received bytes

Using -R, you can have Loramon print out all the bytes that it receives prior to decoding them.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C -R
```
Loramon will then print out hex bytes. For example, the following shows the battery message in raw form and decoded form.

```
-->0xc0 0x27 0x01 0x00 0xc0<--
Radio reporting battery state is 1, % 0
```

### Capture packets for a specified duration

Using --duration, you can have Loramon start up, scan for packets, and exit after the given time.  This allows you to run a wrapper script for scanning multiple channels for activity.  In this case, Loramon will return the number of packets it captured.
Note: In Linux, the return code is 1 byte. The maximum reported nubmer for captured packets will be 255.

```sh
loramon.py /dev/ttyUSB0 --freq 914875000 --bw 125000 --sf 8 --cr 5 -C --duration 5 ; echo Number of captured packets \: $?
```

Loramon will return the number of captured packets and the next statement will print it.

```
[2025-04-25 19:53:56] Opening serial port /dev/ttyUSB0...
[2025-04-25 19:53:59] Radio reporting frequency is 914.875 MHz
[2025-04-25 19:53:59] Radio reporting bandwidth is 125.0 KHz
[2025-04-25 19:53:59] Radio reporting TX power is 0 dBm
[2025-04-25 19:53:59] Radio reporting spreading factor is 8
[2025-04-25 19:53:59] Radio reporting coding rate is 5
[2025-04-25 19:53:59] Radio reporting radio state is 1
[2025-04-25 19:53:59] Radio reporting radio state is 1
[2025-04-25 19:53:59] Radio reporting promiscuous mode is 0
[2025-04-25 19:54:00] Capture Duration 5
[2025-04-25 19:54:05] Radio reporting rssi is -110
[2025-04-25 19:54:05] Radio reporting snr is 5.5
[2025-04-25 19:54:05] [-110 dBm] [SNR 5.5 dB] [211 bytes]       b"\x0c\x00*\xea\x14\x9fe\\\xcb\xb7}\xa6\xb4\xf5L7\xc4\x12\x00 Y\xe1MOK\x15\x14o\xd7DQ\xdc\xa7\x9b\\\xd7<\x91Pb\x87L\xb4E{\xf9bL7\xb4u\xb9\xff5\x8c\xd0c\xa9]_Vo\x1ds\xd7\x85OX\x90\xdd\x94!i\xb7\x0b~\xdb\x08\xc83\xaf\xb1\\b\xba\xe7\xfb\xb1\x90\xde\x18\x17\xf4\xe0\x0bE\xe1\x1dXb\xcbP,\x98-U\x8e{\xfb5\x96?F0W*#\xe2\x05\x88W\xfb\x96\xa3\xc7\xdd\xb5\xd3\x86\xb1\xa5\x08\xfc\x1e\xd0i\xd0~ ;\x87q\x80\xbe\x96\x1c\xee\xa3\x82\x8a'\xbb\xd3\xbb\xb3\t\xae\x92\xb4SQ\xf1\xfe\x88\xa4\xba\xebx\xbf]\xe6#m\x82\xa9/\xb4U\xdbD4\x0e\xe6r\xfa\xfe\x85\xef\xdf\xd8\xa1\xee)\xc4\x91d\xb6\xe6B\xf7\xa6\x81;\x14\xee\xcd3\x1bmV\x81"
[2025-04-25 19:54:05] Radio reporting rssi is -116
[2025-04-25 19:54:05] Radio reporting snr is -2.0
[2025-04-25 19:54:05] [-116 dBm] [SNR -2.0 dB] [51 bytes]       b'\x08\x00k\x9ff\x01M\x98S\xfa\xab"\x0f\xbaG\xd0\'a\x00\x19eE\xa4\x9a\xa8\xb8\xb8V\xb7\x1c\xe3\xf6a\xc2\xf8\xefRk\xf3\x1b\x07I\x9d\xec~\xd9\x8c\x84\xf7\x1f\xba'

Number of captured packets : 2
```

### Quiet Mode

Using -Q, you can supress all messages that are printed to the console.  The main purupose of this mode is reducing chatter when --duration is used.

```sh
loramon.py /dev/ttyUSB0 --freq 914875000 --bw 125000 --sf 8 --cr 5 -C --duration 5 -Q ; echo Number of captured packets \: $?
```

will result in a single line being printed to the console:

```
Number of captured packets : 3
```

### UI Mode

Using -U you can enable the experimental UI mode.  The UI was written in python, using urwid text library; which is the same one used for Nomadnet.
UI mode allows more interaction with Loramon while it is running.  For example, you can
- Change the frequence
- Change the bandwidth
- Change the spread factor
- Change the coding rate
- Turn raw mode on / off

The goal is to have the UI function as dashboard for monitoring the KISS protocol, as well as packets, and other information about the Radio.
