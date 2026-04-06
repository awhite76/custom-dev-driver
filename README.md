# Custom RetroPie Joystick Driver

A custom Raspberry Pi / RetroPie input driver that:

- reads **two thumbsticks** through an **ADS1015** over I2C
- scans a **button matrix** using GPIO
- exposes everything as a **virtual Linux joystick** using `uinput`
- can be started automatically at boot with `systemd`

This is useful for handheld RetroPie builds, custom consoles, and DIY game controllers.

---

## Features

- Dual analog stick support
- GPIO button matrix scanning
- Virtual gamepad exposed through `/dev/uinput`
- Works with RetroPie / EmulationStation
- Can start automatically on boot
- Supports custom GPIO wiring

---

## Hardware Used

### Analog input
- **ADS1015** 4-channel ADC on I2C
- 4 analog channels used:
  - left stick X/Y
  - right stick X/Y

### GPIO buttons
This driver scans a **2x4 button matrix** using Raspberry Pi GPIO pins.

#### Matrix outputs
- `Lout` → GPIO 26
- `Rout` → GPIO 23

#### Matrix inputs
- `Uin` → GPIO 24
- `Rin` → GPIO 25
- `Din` → GPIO 5
- `Lin` → GPIO 6

### Thumbstick click buttons
- Left thumb button → GPIO 22
- Right thumb button → GPIO 27

> Note: in the current code, GPIO 22 and GPIO 27 are requested but not yet emitted as joystick buttons.

---

## Button Mapping

The matrix maps to these virtual joystick buttons:

| Physical Button | Virtual Button |
|---|---|
| Up | `BTN_START` |
| Right | `BTN_TR` |
| Down | `BTN_SELECT` |
| Left | `BTN_TL` |
| Y | `BTN_Y` |
| B | `BTN_B` |
| A | `BTN_A` |
| X | `BTN_X` |

Analog sticks map as:

| Stick | Axis |
|---|---|
| Left X | `ABS_X` |
| Left Y | `ABS_Y` |
| Right X | `ABS_RX` |
| Right Y | `ABS_RY` |

---

## Dependencies

Install the required packages:

```bash
sudo apt update
sudo apt install -y build-essential libgpiod-dev i2c-tools libi2c-dev
