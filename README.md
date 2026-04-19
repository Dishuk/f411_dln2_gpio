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

## Layout

```
Core/                        CubeMX-generated, has USER CODE blocks
USB_DEVICE/App/
  usb_device.c               CubeMX; our PostTreatment hook registers DLN-2
  usbd_dln2.c/.h             DLN-2 class + GPIO command handlers
  usbd_dln2_desc.c/.h        VID/PID/strings for a257:2013
f411_dln2_gpio.ioc           CubeMX config — source of truth
Drivers/, Middlewares/       .gitignored; regenerated from .ioc
```

## Capabilities

- 17 GPIO lines (PC13 + PB0–PB15).
- Per-line direction (input with pull-up, or output push-pull).
- Per-line value read and write.
- Pin state is latched across `gpioset` invocations — releasing the line
  on the host does not reset the pin.
