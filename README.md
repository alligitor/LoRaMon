# LoRa packet sniffer for RNode hardware

## Intro

This utility allows you to sniff LoRa networks with an [RNode](https://unsigned.io/projects/rnode/), and dump captured packets to the console or files.

```sh
usage: loramon [-h] [-C] [-W directory] [--freq Hz] [--bw Hz] [--txp dBm]
               [--sf factor] [--cr rate]
               [--duration seconds] [-Q]
               [-R]
               [-P]
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
  -Q             Quite mode. Don't log any messages after start up
  -R             Print out raw bytes of the frame
  -P             Set promiscuous mode
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
