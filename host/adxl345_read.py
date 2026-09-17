#!/usr/bin/env python3
"""Read an ADXL345 through spidev (standard library only).

Usage: adxl345_read.py /dev/spidevN.0 [samples]
"""

import ctypes
import fcntl
import struct
import sys
import time

SPI_IOC_MESSAGE_1 = 0x40206B00  # _IOW('k', 0, struct spi_ioc_transfer)

REG_DEVID = 0x00
REG_POWER_CTL = 0x2D
REG_DATA_FORMAT = 0x31
REG_DATAX0 = 0x32


def xfer(fd, data):
    tx = ctypes.create_string_buffer(bytes(data), len(data))
    rx = ctypes.create_string_buffer(len(data))
    # struct spi_ioc_transfer: tx_buf, rx_buf, len, speed_hz, delay_usecs,
    # bits_per_word, cs_change, tx_nbits, rx_nbits, word_delay_usecs, pad
    msg = struct.pack("QQIIHBBBBBB", ctypes.addressof(tx), ctypes.addressof(rx),
                      len(data), 0, 0, 0, 0, 0, 0, 0, 0)
    fcntl.ioctl(fd, SPI_IOC_MESSAGE_1, msg)
    return rx.raw


def read(fd, reg, n=1):
    # bit7 = read, bit6 = multi-byte
    cmd = 0x80 | (0x40 if n > 1 else 0) | reg
    return xfer(fd, [cmd] + [0] * n)[1:]


def write(fd, reg, value):
    xfer(fd, [reg, value])


def main():
    dev = sys.argv[1]
    samples = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    with open(dev, "r+b", buffering=0) as f:
        fd = f.fileno()
        devid = read(fd, REG_DEVID)[0]
        print(f"DEVID = 0x{devid:02X} ({'OK' if devid == 0xE5 else 'expected 0xE5'})")
        if devid != 0xE5:
            sys.exit(1)

        write(fd, REG_DATA_FORMAT, 0x08)  # 4-wire SPI, full resolution, +-2 g
        write(fd, REG_POWER_CTL, 0x08)    # measurement mode
        time.sleep(0.1)

        for _ in range(samples):
            x, y, z = struct.unpack("<hhh", read(fd, REG_DATAX0, 6))
            # full resolution: 3.9 mg per LSB
            print(f"x={x * 0.0039:+.3f} g  y={y * 0.0039:+.3f} g  z={z * 0.0039:+.3f} g")
            time.sleep(0.2)


if __name__ == "__main__":
    main()
