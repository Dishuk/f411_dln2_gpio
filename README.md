# f411_dln2_gpio

STM32F411 firmware that pretends to be a Diolan DLN-2 USB adapter. Linux
binds its built-in `dln2` driver and exposes 17 GPIO lines. Each line
can be configured as input or output, driven high or low, and read
back — through standard `gpioset` / `gpioget`. No custom host driver.

Inspired by [picoports](https://sevenlab.de/2025/09/23/picoports/).

## Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) | 1.13+ | Edit `.ioc`, build, flash |
| STM32Cube FW_F4 | V1.28.3 | HAL + USB library |
| ST-Link or DFU | — | Flashing |
| Linux (3.19+) | — | Has the `dln2` kernel driver |
| libgpiod | v1 or v2 | `gpioset`, `gpioget` |

## Pin mapping

| Line | Pin    | Note                    |
|-----:|--------|-------------------------|
| 0    | PC13   | Onboard LED, active-low |
| 1–16 | PB0–PB15 | —                     |

## Setup

### 1. Regenerate the CubeMX tree

`Drivers/`, `Middlewares/`, linker scripts and startup assembly are
`.gitignore`d. Open `f411_dln2_gpio.ioc` in STM32CubeIDE and click
**GENERATE CODE** once after cloning.

### 2. Build

`Ctrl+B`. Expect 0 errors, 0 warnings, ~30 KB flash.

### 3. Flash

Via **ST-Link**: wire SWDIO=PA13, SWCLK=PA14, GND, 3V3. Click Run in
CubeIDE.

Via **DFU**: hold `BOOT0`, tap `NRST`, release `BOOT0`. Open
STM32CubeProgrammer, switch to USB, connect, program
`Debug/f411_dln2_gpio.elf`.

### 4. Check it works

Plug the board's own USB-C port into a Linux host:

```
lsusb | grep a257          # a257:2013 Diolan DLN-2
gpiodetect                 # gpiochipN [dln2] (17 lines)
gpioset gpiochipN 0=0      # LED on (PC13 is active-low)
gpioset gpiochipN 0=1      # LED off
gpioget gpiochipN 1        # read PB0
```

## SPI

SPI1 is exposed as a DLN-2 SPI master with one chip select. The kernel's
`dln2-spi` driver registers it as `spiN`.

| Signal | Pin (board label) |
|--------|-------------------|
| CS0    | PA4 (`A4`)        |
| SCK    | PA5 (`A5`)        |
| MISO   | PA6 (`A6`)        |
| MOSI   | PA7 (`A7`)        |

8-bit frames, modes 0–3, 375 kHz – 48 MHz (96 MHz / 2..256, rounded down).

A USB adapter can't describe what's wired to it, so Linux needs a small
module to create the SPI device. `host/dln2_adxl345/` registers an ADXL345
on CS0 (mode 3, 1 MHz) and binds it to spidev:

```
apt install proxmox-headers-$(uname -r) gcc make   # or linux-headers-* elsewhere
make -C host/dln2_adxl345
modprobe spidev
insmod host/dln2_adxl345/dln2_adxl345.ko           # -> /dev/spidevN.0
python3 host/adxl345_read.py /dev/spidevN.0        # DEVID = 0xE5, x/y/z in g
```

## Layout

```
Core/                        CubeMX-generated, has USER CODE blocks
USB_DEVICE/App/
  usb_device.c               CubeMX; our PostTreatment hook registers DLN-2
  usbd_dln2.c/.h             DLN-2 class + GPIO and SPI command handlers
  usbd_dln2_desc.c/.h        VID/PID/strings for a257:2013
f411_dln2_gpio.ioc           CubeMX config — source of truth
Drivers/, Middlewares/       .gitignored; regenerated from .ioc
host/dln2_adxl345/           Linux module: ADXL345 spidev on the DLN-2 SPI bus
host/adxl345_read.py         Reads the ADXL345 through spidev
```

## Capabilities

- 17 GPIO lines (PC13 + PB0–PB15).
- Per-line direction (input with pull-up, or output push-pull).
- Per-line value read and write.
- Edge and level events on every line, so `gpiomon` works. Lines are
  sampled in the main loop (no EXTI: PC13 and PB13 share EXTI line 13), so
  pulses shorter than one loop pass can be missed. With `-r` or `-f` the
  edge type is exact; when watching both edges, the kernel reads the line
  again to label each event, so very fast changes may be mislabelled.
- SPI master on SPI1 with one chip select (see [SPI](#spi)).
- Pin state is latched across `gpioset` invocations — releasing the line
  on the host does not reset the pin.
