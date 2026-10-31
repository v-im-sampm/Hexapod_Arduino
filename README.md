# ESP32 / PCA9685 hexapod

Firmware for the [original printed hexapod](https://makerworld.com/en/models/1973965-hexapod-spider-robot-with-servo-arduino#profileId-2122819)
([assembly/video](https://www.youtube.com/watch?v=ciE2Z_92T7s)), using:

- An ESP-WROOM-32 development board (Arduino board selection: **ESP32 Dev Module**).
- Two PCA9685 16-channel servo drivers, addressed **0x40** and **0x41**.
- Eighteen **positional MG996R** servos, three per leg (not continuous-rotation servos).
- The CyberBrick ESP-NOW transmitter in [espnow/tx/tx.py](espnow/tx/tx.py).

The original leg geometry, inverse kinematics, alternating-tripod gait, step dimensions and smoothing are the defaults. All motion/calibration parameters can be adjusted over the serial console and explicitly saved to ESP32 flash. Direct GPIO servo output, Mega pin assignments, SPI/nRF24L01 control and unused forward-kinematics intermediates have been removed. The ESP32's built-in radio replaces the nRF24L01; no separate radio module is connected.

## 1. Power first

**Support the robot with its feet clear of the bench during setup. Boot always leaves outputs disabled; serial commands explicitly enable them. Disabling outputs can release torque and collapse the robot. Keep a physical servo-power disconnect within reach.**

- Use a regulated **5-6 V** servo supply; 6 V is a suitable nominal choice. Never connect a 2S LiPo directly to the servos. Use a suitably rated regulator/BEC.
- Size the supply, distribution wiring, connectors and fuses for the actual load. [TowerPro's MG996R specification](https://towerpro.com.tw/product/mg996r/) lists 1.4 A stall current: eighteen servos could demand **25.2 A**. A 6 V / 30 A supply is a starting point for those specifications, not a guarantee for clones or a loaded robot. Measure your units; some variants draw substantially more. Avoid prolonged stalls.
- **Do not power servos from ESP32 USB, VIN, 5V or 3V3, a breadboard, or thin daisy-chain cables.**
- The recommended wiring below bypasses the PCA9685 servo-power traces: distribute power directly to each servo through a high-current harness. PCA9685 board/terminal ratings vary; do not assume one board can carry nine MG996Rs' current.
- Power the ESP32 development board separately through USB or its documented regulated input. A bare ESP-WROOM-32 module needs its own regulated 3.3 V supply and programming circuitry; it cannot take USB/5 V directly.
- Join ESP32 ground, both PCA9685 grounds, and servo-supply negative at a common ground. Run servo return currents directly to the power distribution, not through the ESP32.
- Place bulk capacitance near the servo distribution (for example, start with 1000 uF / 10 V or higher per group of nine, observing polarity). Capacitors do not replace adequate supply capacity or wiring.

## 2. Daisy-chain the PCA9685 logic

Disconnect power before changing wiring or soldering jumpers. GPIO numbers below are the ESP32's **GPIO labels**, not header positions.

| ESP32 connection | PCA9685 #1 | PCA9685 #2 |
|---|---|---|
| 3V3 | VCC | VCC, chained from #1 |
| GND / common ground | GND | GND, chained from #1 |
| GPIO21 | SDA | SDA, chained from #1 |
| GPIO22 | SCL | SCL, chained from #1 |
| GPIO23 | OE | OE, chained from #1 |
| Not connected | V+ | V+ |
| Address jumpers | A0-A5 all open: **0x40** | **A0 bridged**, A1-A5 open: **0x41** |

**VCC is logic power (3.3 V); V+ is the separate servo-power rail. They are not interchangeable.** With the recommended external servo-power harness, leave both V+ terminals/header rails disconnected. PWM generation only needs VCC.

The chain is an electrically **parallel I2C bus**, not a serial data-output/data-input connection. Keep it short; the defaults are 400 kHz I2C and 50 Hz servo PWM. Breakout boards normally include SDA/SCL pull-ups to VCC. Ensure all pull-ups go to **3.3 V**, never 5 V.

OE is active-low: GPIO23 LOW enables pulses; HIGH disables both boards. Add an **external 1 kOhm pull-up from the shared OE line to 3V3** so outputs stay disabled while the ESP32 resets. Many boards have 10 kOhm OE pull-downs; the 1 kOhm pull-up overcomes two of these. Check your board variant and confirm OE stays high during reset; remove conflicting pull-downs if needed. Do not tie OE to GND. On a detected initialization or servo-write fault, firmware drives OE high. The console remains available: fix the fault, then use `init` to retry without reflashing.

### Wiring diagram

```text
                   LOGIC ONLY (3.3 V)
 ESP32 DevKit             PCA9685 #1                PCA9685 #2
                         address 0x40              address 0x41
 USB --> ESP32           A0-A5 open                 bridge A0 only
 3V3 -------------------- VCC ---------------------- VCC
 GND ----+--------------- GND ---------------------- GND
 GPIO21 -|--------------- SDA ---------------------- SDA
 GPIO22 -|--------------- SCL ---------------------- SCL
 GPIO23 -|-----+--------- OE ----------------------- OE
         |     |
         |     +---[1 kOhm]--- ESP32 3V3
         |
         |               V+ NOT CONNECTED           V+ NOT CONNECTED
         |               S0-S8 --> right legs       S0-S8 --> left legs
         |
         +--------------------------- COMMON GROUND
                                      |
 Servo supply (-) --------------------+---- heavy return harness
                                                |
                                                +--> ALL servo brown/black wires

 Regulated 5-6 V servo supply (+)
         |
    fuse / power disconnect
         |
    rated, fused distribution ----------------------> ALL servo red wires

 EACH SERVO:
   orange/yellow (signal) <--- assigned PCA9685 channel S/PWM pin
   red           (+)     <--- external servo-power distribution
   brown/black   (-)     <--- external heavy ground/return distribution
```

Use servo extension/splitter leads that separate signal from power; insulate exposed contacts. Read the board's **S / V+ / GND** markings rather than relying on connector orientation or colors alone. In this arrangement, only the signal lead goes to the channel header; the servo's ground reference is provided by the common-ground harness.

If instead using conventional three-wire plugs directly on the PCA9685, first verify that its terminals, traces and headers can safely carry the measured load. Feed each board's V+ and GND separately with rated, fused power wiring; never pass the second board's servo current through the first board or the logic chain. The external power harness above is preferred for eighteen MG996Rs.

## 3. Connect all eighteen servos

Left/right are from the robot's perspective, looking forward. The coordinate convention is +X forward, +Y left.

```text
                         FRONT (+X)
                             ^
        LF (leg 5)      \    |    /      RF (leg 0)
                        +-------+
 LEFT   LM (leg 4) ------| BODY  |------ RM (leg 1)   RIGHT
 (+Y)                   +-------+
        LR (leg 3)      /         \      RR (leg 2)
                             REAR

 Each leg: BODY -- coxa (hip yaw) -- femur (hip lift) -- tibia (knee) -- FOOT
```

Channel numbers are the **printed, zero-based PCA9685 channel labels**, not ESP32 GPIO numbers.

| Leg | Firmware index | Board / address | Coxa signal | Femur signal | Tibia signal |
|---|---:|---|---:|---:|---:|
| Right front (RF) | 0 | #1 / 0x40 | 0 | 1 | 2 |
| Right middle (RM) | 1 | #1 / 0x40 | 3 | 4 | 5 |
| Right rear (RR) | 2 | #1 / 0x40 | 6 | 7 | 8 |
| Left rear (LR) | 3 | #2 / 0x41 | 0 | 1 | 2 |
| Left middle (LM) | 4 | #2 / 0x41 | 3 | 4 | 5 |
| Left front (LF) | 5 | #2 / 0x41 | 6 | 7 | 8 |

Channels **9-15 on both boards are unused and disabled**. Every servo's red and brown/black wires connect to the external power harness described above, regardless of its signal channel.

## 4. Build and upload

1. Install Arduino IDE 2.x and **esp32 by Espressif Systems**, version **3.x** (the receive callback uses the current ESP-IDF API, not the old 2.x signature).
2. Select **ESP32 Dev Module** for an ESP-WROOM-32 development board, with its actual flash size (commonly 4 MB) and serial port.
3. Install **Adafruit PWM Servo Driver Library** and its **Adafruit BusIO** dependency using Library Manager. WiFi, Wire, ESP-NOW, Preferences/NVS and FreeRTOS come with the ESP32 core. No Servo, ESP32Servo or RF24 library is needed.
4. Open [Hexapod_Arduino.ino](Hexapod_Arduino.ino), keeping [HexapodControl.h](HexapodControl.h) and [HexapodConsole.h](HexapodConsole.h) beside it.
5. Upload once; no source editing or reflashing is needed for normal calibration or parameter tuning.
6. Upload with servo power off. Open Serial Monitor at **115200 baud**, select **Newline** or **Both NL & CR**, then reset. Boot loads saved settings if valid, initializes the drivers, and stays **off**. Type `help`. The receiver's **STA MAC address** is printed before PCA9685 initialization, so it can be read even with the drivers disconnected. Opening the serial port may reset some development boards; this also returns outputs to off.

### Flash and open the console from VS Code

Connect the ESP32 over USB, keep servo power disconnected, and close any other application using its COM port. Find the port under **Device Manager > Ports (COM & LPT)**.

Choose **Terminal > Run Task > Hexapod: Flash + Serial Console**, then enter the port, for example `COM5`. The task:

1. Builds the current sketch for **ESP32 Dev Module**.
2. Uploads those freshly built binaries to the COM port you supplied.
3. Opens an **interactive 115200-baud serial console** in the same focused terminal, only if the upload succeeded.

Type `help` and press **Enter**. Commands and replies use that terminal; this is not a read-only output window. Send `off` and Enter before closing the console with **Ctrl+C**. Closing a monitor alone does not disable a robot already running in radio mode or holding a pose. Close the old monitor before starting another upload.

The tasks in [.vscode/tasks.json](.vscode/tasks.json) launch **Windows PowerShell directly**, even if your default VS Code terminal is WSL. They do not change your default terminal profile. Available tasks:

| Task | Purpose |
|---|---|
| **Hexapod: Build** | Build without a connected board; also available through Ctrl+Shift+B |
| **Hexapod: Flash + Serial Console** | Prompt for COM port, build, upload, then accept serial input |
| **Hexapod: Serial Console** | Prompt for COM port and reconnect without building/flashing |

Alternatively, in a **Windows PowerShell** terminal opened at the repository root:

```powershell
.\tools\flash.ps1 -Port COM5
```

Replace `COM5` with the actual port; omit `-Port` to be prompted. The same script supports:

```powershell
.\tools\flash.ps1 -BuildOnly
.\tools\flash.ps1 -MonitorOnly -Port COM5
```

If local script execution is blocked, use the VS Code tasks or this process-only invocation; no machine-wide execution-policy change is needed:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Port COM5
```

[tools/flash.ps1](tools/flash.ps1) finds `arduino-cli` on PATH or in the standard Arduino IDE 2.x installation locations. It reuses `%USERPROFILE%\.arduinoIDE\arduino-cli.yaml` when present, including the IDE's existing board packages and libraries. Without that file, the CLI uses its default configuration. Nonstandard installations can pass `-ArduinoCli "C:\path\arduino-cli.exe"` and/or `-ConfigFile "C:\path\arduino-cli.yaml"`.

Build artifacts are cached in [.build/esp32](.build/esp32), which Git ignores. Build or upload failures stop the workflow instead of opening a misleading console or flashing stale artifacts. If only the monitor failed to open, fix the port/connection and use `-MonitorOnly`; a successful upload need not be repeated. The workflow does not request a full-flash erase, so existing NVS settings normally survive uploading.

The terminal monitor is **line-buffered**, allowing normal local typing/editing: press Enter to send a command. In this mode **`!` also needs Enter**; it is not an immediate keystroke emergency stop. Keep the physical power disconnect accessible.

For a standalone Arduino CLI installation on PATH, install the prerequisites once if not already present:

```powershell
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.12 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit PWM Servo Driver Library@3.0.3"
```

If a board does not enter download mode automatically, use its BOOT/EN buttons as directed by its manufacturer. Do not flash the reference MicroPython receiver onto the hexapod's ESP32.

## 5. Pair the ESP-NOW transmitter

The default protocol matches [tx.py](espnow/tx/tx.py) and [rxbulldozer.py](espnow/rx/bulldozer/rxbulldozer.py): unencrypted ESP-NOW on **Wi-Fi channel 1**, station mode, about one packet every **20 ms**. No router, SSID or Wi-Fi password is needed. Do not connect either device to an access point that changes its channel. If changing `radio_channel`, also change the transmitter's channel; the receiver cannot remotely reconfigure the transmitter.

1. Copy the printed **receiver STA MAC** into the desired `receiver1_mac`, `receiver2_mac` or `receiver3_mac` entry in [tx.py](espnow/tx/tx.py). For a single robot, set all three to that same MAC; otherwise L1 selects which model receives commands.
2. For example, a receiver MAC of `24:6f:28:12:34:56` becomes `b'\x24\x6f\x28\x12\x34\x56'`. Use your actual address, not this example.
3. On the **CyberBrick transmitter**, copy [boot.py](espnow/tx/boot.py), [main.py](espnow/tx/main.py) and your configured [tx.py](espnow/tx/tx.py). Keep the empty boot script; the main script starts the transmitter. Reset it.
4. After calibration, send `radio` from the serial console to enable the home stance and select ESP-NOW control. Center the left vertical and right horizontal sticks. Serial Monitor reports `ESP-NOW control armed.` before walking commands are accepted. `status` shows the latest valid received CSV packet and its age, even while outputs are off.

The payload is ASCII/UTF-8 CSV, **not a binary array of integers**, with no trailing newline or NUL:

```text
L1,L2,L3,R1,R2,R3,K1,K2,K3,K4
0,2047,2047,2047,2047,2047,1,1,1,1
```

| Zero-based field | Input | Accepted range | Hexapod use |
|---:|---|---|---|
| 0 | L1 / model switch | 0-4095 | Transmitter selects receiver; ignored here |
| 1 | L2 / left horizontal | 0-4095 | Unused |
| 2 | L3 / left vertical | 0-4095 | Walk forward/backward |
| 3 | R1 / slider | 0-4095 | Unused |
| 4 | R2 / right horizontal | 0-4095 | Turn left/right |
| 5 | R3 / right vertical | 0-4095 | Unused |
| 6-9 | K1-K4 / buttons | 0 or 1 (active-low) | Unused |

By default, both used sticks have a neutral interval of **1947-2147 inclusive** (2047 +/- 100, as in the bulldozer receiver). Throttle below 1947 means forward; above 2147 means reverse. Steering below 1947 requests positive rotation/left; above 2147 requests negative rotation/right. Use `set throttle_reverse 1` or `set steering_reverse 1` while off if the physical direction differs. Walking and turning may be combined; radio control remains discrete rather than proportional, with configurable stride and timing.

Exactly ten unsigned decimal fields are validated: up to four digits per analog field and one per button, at most 37 bytes total. Malformed packets are counted/reported and do not refresh the watchdog. Other channels are validated but have no motion effect; buttons are **not** emergency stops.

In radio mode, after **500 ms without a valid packet** by default (`timeout_ms`), motion commands become zero and the feet smoothly return toward their home stance. Servos remain powered and holding; this is not an emergency power cutoff. Entering radio mode and signal recovery require neutral sticks before motion is enabled again. Selecting another model therefore also stops this robot after the timeout. Radio loss does not reboot ESP-NOW. Radio packets never enable outputs at boot or override serial tests; use `radio` explicitly to hand control back.

Like the supplied examples, this receiver accepts valid packets from any sender that addresses it on the correct channel; it does not authenticate or encrypt commands. Use it in a controlled environment.

## 6. Center and calibrate the MG996Rs

MG996R travel and pulse limits vary, especially among clones. Do **not** assume every unit safely supports 500-2500 us or a full 180-degree sweep.

1. Disconnect servo power, support the body, and detach horns/linkages as necessary to prevent binding. Boot with the console connected; outputs remain disabled.
2. Power and check **one servo at a time**: `servo rf coxa 90` enables only that joint on a fresh off-to-pose transition. `center rf` centers its three joints; `center all` centers all eighteen. Subsequent pose commands retain other already-enabled joints. Use `off` between tests to disable everything and start afresh.
3. Defaults are conservative **1000-2000 us at 50 Hz**, giving **1500 us at nominal 90 degrees**. Fit horns in the original model's assembly/neutral orientation, then connect linkages. Do not reposition an energized servo by force.
4. Use small explicit angle or pulse steps, for example `servo rf coxa 95`, `pulse rf coxa 1520`, then `status`. Pulse tests bypass trim/reversal but stay inside that joint's configured pulse limits. There is deliberately no unattended full-range sweep.
5. To adjust a joint, send `off`, then for example `cal rf coxa 1000 2000 5 1`: minimum pulse, maximum pulse, +5-degree trim, reversed direction. Use `servo rf coxa 90` to test the new calibration. Change limits conservatively, only after measuring actual travel/clearance; the software's allowed 500-2500 us envelope is **not** mechanically safe for every unit.
6. Repeat for each joint. Then send `off` and `save`. Calibration and the other settings now survive reset/power loss.
7. Use `home` to enable the home stance, then a short, reduced-stride test such as `walk 0.25 0 1000`. This walks for one second without a transmitter, then returns toward home. Test turning with `walk 0 0.25 1000`. Send `stop` to return home early, or `off`/`!` to disable pulses and release torque.
8. Send `radio`, center the sticks, and test the radio failsafe by turning the transmitter off. Stop immediately for binding, buzzing, overheating, supply dips or resets.

The gait assumes calibrated physical joint angles. A conservative 1000-2000 us range may produce less travel and is **not** a verified full-range calibration for your MG996Rs. If a servo cannot cover the required travel, adjust the mechanism/gait rather than forcing it against its stops. Nominal angles are clamped to 0-180 after reversal/trim. Gait coxa limits default to 45-135 degrees; direct `servo`/`joints` tests intentionally permit 0-180 for calibration.

### Automated one-servo-at-a-time test

After manually checking each joint's neutral position and clearance, support the robot with its feet clear and keep the physical servo-power disconnect within reach. Send these commands individually:

```text
off
testservos
```

`testservos` requires off mode and initialized drivers. It tests all eighteen joints in wiring order: **rf, rm, rr, lr, lm, lf**, with **coxa, femur, tibia** for each leg. Only the current joint receives pulses; previously tested joints and unused channels are disabled. Each joint moves through nominal **90 -> 85 -> 95 -> 90 degrees**, holding each angle for **1000 ms** before advancing. The existing pulse limits, trim and reversal apply. This is a small-range check, not an endpoint sweep or automatic calibration; even these angles must be checked for mechanical clearance first.

The console reports each joint, board address, channel, nominal angle and commanded pulse width. `status` remains available. The non-blocking sequence normally takes about **72 seconds**; delayed loop servicing extends holds rather than skipping movements. These are commanded positions, not measured feedback, and completion does not certify that a servo moved correctly.

Send `off` or `!` to cancel and disable outputs, or `stop` to cancel while retaining the current joint's holding torque. **Normal completion disables all outputs and releases torque**, so keep the body supported throughout. Radio packets never interrupt or resume the sequence; `home`, `walk` or `radio` explicitly cancel it and enable all legs, which can move joints abruptly. Direct pose commands and configuration require leaving the sequence first. An I2C fault disables outputs and requires `init` before restarting. The sequence does not change or save settings.

## 7. Serial console reference

Commands are case-insensitive, space/tab separated, and terminated by LF, CR or CRLF. Leg identifiers are `rf rm rr lr lm lf` or `0..5`; joints are `coxa femur tibia` or `0..2`. Numbers must be finite and in range; malformed commands do not partially apply settings. Maximum line length is 159 characters. Overlong/invalid lines are discarded through the next line ending, without stopping the control loop.

`!` disables outputs as soon as that byte reaches the firmware, without waiting for a line ending. Finish its line before the next command. The supplied terminal monitor buffers keyboard input until **Enter**, so `!` still requires Enter there. It is a software disable, not a hardwired emergency stop. `off` also releases torque; `stop` does not.

| Command | Effect |
|---|---|
| `help` | Command summary (while off) |
| `status` | Mode, OE, driver/radio readiness, unsaved changes, MAC, latest RX values/age; while off/posing/testing servos, commanded pulses and current sequence step (not position feedback) |
| `params` | All named parameter ranges (while off) |
| `show` | All current parameters, leg origins and eighteen calibrations, displayed as commands (while off) |
| `off` / `!` | Disable both PCA9685 outputs immediately; radio cannot re-enable them |
| `stop` | Stop radio/manual walking and return toward home, retaining holding torque; leave a direct pose or off state unchanged |
| `center [all\|LEG]` | Hold nominal 90 degrees; default is all joints |
| `servo LEG JOINT ANGLE` | Test one nominal angle, 0-180 degrees |
| `pulse LEG JOINT US` | Test an integer pulse width within that servo's calibrated limits |
| `testservos` | While off, test all eighteen joints one-by-one at nominal 90/85/95/90 degrees, 1000 ms per angle; disable outputs on completion; `stop` cancels holding the current joint |
| `joints LEG COXA FEMUR TIBIA` | Set all three nominal angles of one leg |
| `foot LEG DX DY DZ` | Test one foot via IK; offsets from that leg's home in body coordinates, each -150..150 mm; unreachable targets/joint angles are rejected |
| `home` | Enable all legs at the home stance; ignore radio |
| `walk DIRECTION ROTATION DURATION_MS` | Serial walking/turning test; each command axis -1..1; mandatory duration **100..10000 ms**, then return toward home |
| `radio` | Enable home stance, select radio, discard queued commands and require neutral sticks before arming |
| `set NAME VALUE` | Change one named parameter in RAM (while off) |
| `cal LEG JOINT MIN_US MAX_US TRIM REVERSE` | Set a complete joint calibration: `500 <= MIN_US < MAX_US <= 2500`, trim -90..90 degrees, reverse 0/1 (while off) |
| `origin LEG X Y` | Set a leg's body mounting location, each coordinate -500..500 mm, at least 1 mm from body center (while off) |
| `save` | Write all settings to flash and verify read-back (while off) |
| `load` | Restore valid saved settings; on error keep current RAM settings (while off) |
| `defaults` | Restore factory defaults in RAM only; follow with `save` to replace saved settings (while off) |
| `init` | Reinitialize radio/drivers with current settings, clear PWM registers, leave outputs off |
| `scan` | Scan the I2C bus while off; normally 0x40/0x41 and possibly 0x70 |

Direct servo/foot tests require **off or pose** mode; send `off` first if home/radio/walking, or cancel `testservos` with `off`/`stop`. The first pose command after off clears stale PWM outputs so unrelated joints do not unexpectedly re-enable. `home`, `walk` and `radio` explicitly select their control source and enable all legs; moving from a direct pose to home can move joints abruptly. Support the robot. Neither completion of a serial test nor a reconnect automatically resumes radio control.

Long diagnostics/configuration/flash operations require off so they cannot block an active gait. Input processing is bounded and non-blocking; partial lines do not pause the gait, radio watchdog or test deadline. Durations and radio timeouts are serviced by the main loop, not hard-real-time timers.

### Parameters

Lengths/origins/height are in mm; angular parameters are degrees.

| Name | Default | Allowed range |
|---|---:|---|
| `coxa_length` | 38 | 1..300 |
| `femur_length` | 86.25 | 1..300 |
| `tibia_length` | 160.5 | 1..400 |
| `step_length` | 60 | 0..150 |
| `step_height` | 100 | 0..150 |
| `home_z` | -40 | -200..-1; home stance must remain reachable |
| `smoothing` | 0.4 | 0.01..1 |
| `rotation_step` | 15 | 0..45 |
| `phase_step` | 0.015 | 0.001..0.05 cycles per update |
| `update_ms` | 10 | 5..50, integer |
| `coxa_min`, `coxa_max` | 45, 135 | 0..180; min must be less than max |
| `servo_hz` | 50 | 40..60; keep 50 unless deliberately testing pulse timing |
| `oscillator0_hz`, `oscillator1_hz` | 25000000 | 20000000..30000000, integer; board #1/#2 oscillator calibration |
| `i2c_hz` | 400000 | 100000..400000, integer |
| `radio_channel` | 1 | 1..13, integer; must match transmitter and local radio regulations |
| `timeout_ms` | 500 | 100..2000, integer |
| `stick_center` | 2047 | 0..4095, integer |
| `stick_deadzone` | 100 | 1..1024, integer; center +/- deadzone must fit 0..4095 |
| `throttle_channel`, `steering_channel` | 2, 4 | 0..5, integer, distinct analog channels |
| `throttle_reverse`, `steering_reverse` | 0, 0 | 0 or 1 |

Changing `servo_hz`, either oscillator frequency or `i2c_hz` requires `init` before servo output can resume. Changing `radio_channel` requires `init` before radio mode. The console reports pending initialization; geometry, gait and calibration changes need no reinitialization. `load`/`defaults` follow these same rules. `init` never enables outputs.

Default gait period is approximately `update_ms / phase_step`, or 667 ms. Slow I2C settings, diagnostics and processing overhead may limit the achievable update rate. Bounds protect software validity, **not** mechanical safety; inspect clearance throughout a complete supported gait cycle. Link-length/home changes must also satisfy the home stance reach check. Change interdependent parameters in an order that keeps each intermediate configuration valid.

Use a scope to calibrate each PCA9685 oscillator against known commanded pulse widths. PWM conversion uses the configured oscillator and the read-back prescaler. GPIO wiring, PCA9685 addresses and leg/channel assignments stay fixed to the wiring diagram; console tuning does not change physical wiring or the transmitter's firmware.

### Example tuning session

```text
off
status
scan
params
cal rf coxa 1000 2000 0 0
servo rf coxa 90
servo rf coxa 95
off
set step_length 30
set step_height 40
set phase_step 0.01
save
home
walk 0.25 0 1000
stop
off
show
radio
```

Send commands individually and inspect each result. Only increase a test's amplitude/range after checking the previous movement.

### Saved settings

`save` stores the complete validated configuration in one versioned, checksummed **Preferences/NVS** record (`hexapod` namespace, `settings` key), then verifies the stored bytes. No tuning command auto-saves. `load` restores the whole configuration or leaves it unchanged on failure. `defaults` does not erase flash until you explicitly `save`.

Valid settings load automatically on boot, **but output enable state, pose, arming and motion commands are never saved**. Every reset starts off. Missing saved data uses documented defaults; corrupt/incompatible records or storage errors are reported. At boot those errors leave factory defaults active with outputs disabled; at runtime a failed load retains the current settings. A future incompatible settings layout needs a new record version and recalibration/re-saving. Erasing flash/NVS removes saved settings.

## Troubleshooting

- **Missing PCA9685 / disabled outputs:** check VCC=3.3 V, shared ground, SDA/SCL, and A0 on board #2. Both drivers are required. Fix the fault and use `init`. `scan` may also show the PCA9685 all-call address 0x70; that is not a third board.
- **No servo movement:** boot is intentionally off. Check `status`, external servo power and grounds, and the correct S/PWM pins; use a deliberate `servo`, `center`, `home` or `radio` command after calibration.
- **Wrong leg/joint moves:** use the robot-perspective channel table; board #2 starts with the **left rear**, not left front.
- **No remote control:** select `radio`; check receiver STA MAC, matching Wi-Fi channels, neutral sticks and `status`. Radio packets are deliberately ignored for motion in all other modes.
- **Resets/jitter:** check servo current demand, power/ground routing, bulk capacitance, and short I2C wiring. Never join a 5 V pull-up to an ESP32 GPIO.
- **I2C error during motion:** both OEs are driven high; motion stops but the console remains available. Loss of pulses may release servo torque, so the robot can collapse. Fix the fault, then `init`; output remains off until explicitly enabled. A software failsafe or OE is not a substitute for supporting the robot and a physical power disconnect.

## Validation

The firmware targets ESP32 Arduino core **3.3.12** and Adafruit PWM Servo Driver **3.0.3**. [tests/test_control.cpp](tests/test_control.cpp) covers the full ADC ranges, protocol/failsafe, servo mapping/PWM, command parsing/overflow, parameter validation, record round-trips/corruption and timer wraparound. [tests/test_firmware.cpp](tests/test_firmware.cpp) runs the actual sketch against hardware fakes to check serial commands, disabled startup, selective servo enable, the complete one-servo-at-a-time sequence (angle order, 1000 ms holds, calibration, cancellation, radio isolation, control handovers, timer wraparound and I2C failures), saved-setting reboot/load/write failures, control-source transitions, the 10-second walking limit and recovery after I2C faults.

Run the tests from a Visual Studio Developer PowerShell with MSVC installed:

```powershell
cl /nologo /std:c++17 /EHsc /W4 /WX .\tests\test_control.cpp /Fo"$env:TEMP\hexapod-test-control.obj" /Fe"$env:TEMP\hexapod-test-control.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP\hexapod-test-control.exe" }
cl /nologo /std:c++17 /EHsc /W4 /WX /wd4244 /wd4305 /I.\tests\fakes .\tests\test_firmware.cpp /Fo"$env:TEMP\hexapod-test-firmware.obj" /Fe"$env:TEMP\hexapod-test-firmware.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP\hexapod-test-firmware.exe" }
```

The integration test suppresses MSVC's double-to-float warnings for the Arduino gait math; the pure logic test uses all warnings as errors. Compilation and host tests cannot verify physical wiring, flash power-loss behavior, supply capacity, actual servo endpoints, RF reception or the gait on your assembled robot. Perform the supported bench checks above before walking.

The flashing-workflow tests use a fake CLI, so no board is needed and nothing is uploaded:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tests\test_flash.ps1
```

[tests/test_flash.ps1](tests/test_flash.ps1) checks build/upload/monitor ordering, use of the same build artifacts, paths containing spaces, COM validation, failure propagation, monitor-only mode and forwarding typed input to the monitor.

Original authors: Ozgur Toprak Onsoy and Emre Kalem. See [LICENSE](LICENSE). The supplied [ESP-NOW examples](espnow/) are retained as transmitter/protocol references; their bulldozer motor and LED logic is not part of the hexapod firmware.
