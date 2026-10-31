# ESP32 / PCA9685 hexapod

Firmware for the [original printed hexapod](https://makerworld.com/en/models/1973965-hexapod-spider-robot-with-servo-arduino#profileId-2122819)
([assembly/video](https://www.youtube.com/watch?v=ciE2Z_92T7s)), using:

- An ESP-WROOM-32 development board (Arduino board selection: **ESP32 Dev Module**).
- Two PCA9685 16-channel servo drivers, addressed **0x40** and **0x41**.
- Eighteen **positional MG996R** servos, three per leg (not continuous-rotation servos).
- The CyberBrick ESP-NOW transmitter in [espnow/tx/tx.py](espnow/tx/tx.py).

The original leg geometry, inverse kinematics, alternating-tripod gait, step dimensions and smoothing are retained. Direct GPIO servo output, Mega pin assignments, SPI/nRF24L01 control and unused forward-kinematics intermediates have been removed. The ESP32's built-in radio replaces the nRF24L01; no separate radio module is connected.

## 1. Power first

**Support the robot with its feet clear of the bench during setup. Servos move at startup, even before the remote is armed. Keep a physical servo-power disconnect within reach.**

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

The chain is an electrically **parallel I2C bus**, not a serial data-output/data-input connection. Keep it short; the sketch uses 400 kHz I2C and 50 Hz servo PWM. Breakout boards normally include SDA/SCL pull-ups to VCC. Ensure all pull-ups go to **3.3 V**, never 5 V.

OE is active-low: GPIO23 LOW enables pulses; HIGH disables both boards. Add an **external 1 kOhm pull-up from the shared OE line to 3V3** so outputs stay disabled while the ESP32 resets. Many boards have 10 kOhm OE pull-downs; the 1 kOhm pull-up overcomes two of these. Check your board variant and confirm OE stays high during reset; remove conflicting pull-downs if needed. Do not tie OE to GND. On a detected initialization or servo-write fault, firmware drives OE high and stops until reset.

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
3. Install **Adafruit PWM Servo Driver Library** and its **Adafruit BusIO** dependency using Library Manager. WiFi, Wire, ESP-NOW and FreeRTOS come with the ESP32 core. No Servo, ESP32Servo or RF24 library is needed.
4. Open [Hexapod_Arduino.ino](Hexapod_Arduino.ino), keeping [HexapodControl.h](HexapodControl.h) beside it.
5. For first mechanical setup, set `CENTER_SERVOS_ONLY = true` in the sketch and follow the calibration section **before allowing the robot to walk**.
6. Upload with servo power off. Open Serial Monitor at **115200 baud**, then reset. The receiver's **STA MAC address** is printed before PCA9685 initialization, so it can be read even with the drivers disconnected.

CLI equivalent (from the repository directory; replace `COM5` with your port):

```powershell
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.12 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit PWM Servo Driver Library@3.0.3"
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload --fqbn esp32:esp32:esp32 --port COM5 .
```

If a board does not enter download mode automatically, use its BOOT/EN buttons as directed by its manufacturer. Do not flash the reference MicroPython receiver onto the hexapod's ESP32.

## 5. Pair the ESP-NOW transmitter

The protocol matches [tx.py](espnow/tx/tx.py) and [rxbulldozer.py](espnow/rx/bulldozer/rxbulldozer.py): unencrypted ESP-NOW on **Wi-Fi channel 1**, station mode, about one packet every **20 ms**. No router, SSID or Wi-Fi password is needed. Do not connect either device to an access point that changes its channel.

1. Copy the printed **receiver STA MAC** into the desired `receiver1_mac`, `receiver2_mac` or `receiver3_mac` entry in [tx.py](espnow/tx/tx.py). For a single robot, set all three to that same MAC; otherwise L1 selects which model receives commands.
2. For example, a receiver MAC of `24:6f:28:12:34:56` becomes `b'\x24\x6f\x28\x12\x34\x56'`. Use your actual address, not this example.
3. On the **CyberBrick transmitter**, copy [boot.py](espnow/tx/boot.py), [main.py](espnow/tx/main.py) and your configured [tx.py](espnow/tx/tx.py). Keep the empty boot script; the main script starts the transmitter. Reset it.
4. Center the left vertical and right horizontal sticks. Serial Monitor reports `ESP-NOW control armed.` before walking commands are accepted.

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

Both used sticks have a neutral interval of **1947-2147 inclusive** (2047 +/- 100, as in the bulldozer receiver). Throttle below 1947 means forward; above 2147 means reverse. Steering below 1947 requests positive rotation/left; above 2147 requests negative rotation/right. Reverse the corresponding command sign if your transmitter's physical stick orientation differs. Walking and turning may be combined; the original fixed-rate, fixed-stride gait is retained, not proportional speed control.

Exactly ten unsigned decimal fields are validated: up to four digits per analog field and one per button, at most 37 bytes total. Malformed packets are counted/reported and do not refresh the watchdog. Other channels are validated but have no motion effect; buttons are **not** emergency stops.

After **500 ms without a valid packet**, motion commands become zero and the feet smoothly return toward their home stance. Servos remain powered and holding; this is not an emergency power cutoff. Startup and signal recovery require neutral sticks before motion is enabled again. Selecting another model therefore also stops this robot after the timeout. Radio loss does not reboot ESP-NOW.

Like the supplied examples, this receiver accepts valid packets from any sender that addresses it on the correct channel; it does not authenticate or encrypt commands. Use it in a controlled environment.

## 6. Center and calibrate the MG996Rs

MG996R travel and pulse limits vary, especially among clones. Do **not** assume every unit safely supports 500-2500 us or a full 180-degree sweep.

1. Disconnect servo power, support the body, and detach horns/linkages as necessary to prevent binding.
2. Set `CENTER_SERVOS_ONLY = true`, upload, then power and check **one servo at a time** using its assigned channel. This mode holds nominal 90 degrees indefinitely and ignores motion commands.
3. The defaults are conservative **1000-2000 us at 50 Hz**, giving **1500 us at nominal 90 degrees**. Fit the horns in the original model's assembly/neutral orientation, then connect the linkages. Do not reposition an energized servo by force.
4. Adjust the corresponding entry in `SERVO_CALIBRATION` in [Hexapod_Arduino.ino](Hexapod_Arduino.ino). Rows are RF, RM, RR, LR, LM, LF; columns are coxa, femur, tibia. `{}` uses the defaults. An explicit entry is `{1000, 2000, 0.0f, false}`: minimum pulse, maximum pulse, angular trim, reverse direction. For example, use `{1000, 2000, 5.0f, true}` only when that joint needs reversal and +5 degrees of trim.
5. Establish safe endpoints for **each** unloaded servo using a servo tester or controlled bench test. Expand the configured range only after measuring actual angular travel and checking mechanical clearance. The gait's angles assume calibrated physical joint angles; a conservative 1000-2000 us range may produce less travel and is **not** a verified full-range calibration for your MG996Rs. If a servo cannot cover the required joint travel, adjust the mechanism/gait rather than forcing it against its stops.
6. With all horns aligned, set `CENTER_SERVOS_ONLY = false` and upload. All joints first receive 90-degree commands for one second; the firmware then commands the home stance even without radio input. Center the transmitter sticks, test small commands while supported, and verify each joint's direction before ground testing.
7. Turn the transmitter off and confirm walking stops after approximately 500 ms and cannot resume until the sticks are centered. Stop immediately for binding, buzzing, overheating, supply dips or resets.

The output clamps nominal angles to 0-180 degrees after reversal/trim, and preserves the original coxa limit of 45-135 degrees before calibration. Configured pulse endpoints must satisfy `500 <= minUs < maxUs <= 2500`; that software envelope is **not** a claim that all those pulse widths are mechanically safe. Per-board `OSCILLATOR_HZ` can be calibrated with a scope if actual pulse timing differs; PWM conversion uses the configured oscillator and the read-back PCA9685 prescaler.

## Troubleshooting

- **Missing PCA9685 / disabled outputs:** check VCC=3.3 V, shared ground, SDA/SCL, and A0 on board #2. Both drivers are required. Fix the fault and reset. Some I2C scanners also show the PCA9685 all-call address 0x70; that is not a third board.
- **No servo movement:** check external servo power and grounds, OE low when ready, and the correct S/PWM pins. Center-only mode intentionally ignores walking.
- **Wrong leg/joint moves:** use the robot-perspective channel table; board #2 starts with the **left rear**, not left front.
- **No remote control:** check the receiver STA MAC in the selected transmitter slot, channel 1 on both devices, neutral sticks and Serial Monitor diagnostics.
- **Resets/jitter:** check servo current demand, power/ground routing, bulk capacitance, and short I2C wiring. Never join a 5 V pull-up to an ESP32 GPIO.
- **I2C error during motion:** both OEs are driven high and firmware halts. Loss of pulses may release servo torque, so the robot can collapse. A software failsafe or OE is not a substitute for supporting the robot and a physical power disconnect.

## Validation

The firmware targets ESP32 Arduino core **3.3.12** and Adafruit PWM Servo Driver **3.0.3**. Host regression tests in [tests/test_control.cpp](tests/test_control.cpp) exercise all 4096 ADC values in each analog field, malformed packets, exact deadband/500 ms boundaries, timer wraparound, neutral-stick re-arming, all eighteen unique channel assignments, calibration limits and 50 Hz pulse conversion.

Run the tests from a Visual Studio Developer PowerShell with MSVC installed:

```powershell
cl /nologo /std:c++17 /EHsc /W4 /WX .\tests\test_control.cpp /Fo"$env:TEMP\hexapod-test-control.obj" /Fe"$env:TEMP\hexapod-test-control.exe"
if ($LASTEXITCODE -eq 0) { & "$env:TEMP\hexapod-test-control.exe" }
```

Compilation and host tests cannot verify physical wiring, supply capacity, actual servo endpoints, RF reception or the gait on your assembled robot. Perform the supported bench checks above before walking.

Original authors: Ozgur Toprak Onsoy and Emre Kalem. See [LICENSE](LICENSE). The supplied [ESP-NOW examples](espnow/) are retained as transmitter/protocol references; their bulldozer motor and LED logic is not part of the hexapod firmware.
