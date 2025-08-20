# Pico Hysteresis Recorder (MC36xx + RunCam)

## Overview
Firmware for Raspberry Pi Pico / Pico 2 W that:
- Detects motion state (IDLE/WALK/RUN) using an MC36xx accelerometer (EV3635B/MC3635).
- Starts camera recording after 2 consecutive seconds with RUN detected.
- Stops recording after 3 consecutive seconds with no RUN samples.
- Provides USB-CDC logs and LED indication during RUN.

Apps:
- apps/hysteresis_recorder: production firmware (combined motion + camera control)
- apps/activity_detect: motion classification demo (USB logs only)
- apps/camera_control: RunCam UART control demo (start/stop via keyboard)

## Hardware
- Default board: Pico 2 W (rp2350). Change target via CMake if needed.
- MC36xx (EV3635B/MC3635):
  - I2C0 SDA: GP4
  - I2C0 SCL: GP5
  - 3V3 and GND shared with Pico
  - I2C speed: 100 kHz
- RunCam camera UART control:
  - Default UART0 TX: GP0
  - Default UART0 RX: GP1
  - 115200 8N1
  - Alternative (demo-style): UART1 TX GP8 / RX GP9 (change in code)

## Dependencies
- ARM GNU Toolchain (arm-none-eabi-gcc)
- CMake ≥ 3.13
- Pico SDK (auto-fetch via -DPICO_SDK_FETCH_FROM_GIT=ON or set PICO_SDK_PATH)

## Build
```bash
cmake -S . -B build -DPICO_SDK_FETCH_FROM_GIT=ON -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2_w
cmake --build build -j
```
Artifacts:
- build/apps/hysteresis_recorder/hysteresis_recorder.uf2
- build/apps/activity_detect/activity_detect.uf2
- build/apps/camera_control/camera_control.uf2

Flash the UF2 by copying to the Pico’s BOOT drive.

## Project Structure
- apps/
  - hysteresis_recorder/main.c
  - activity_detect/main.c
  - camera_control/main.c
- lib/
  - motion/motion_detect.{c,h}
  - runcam/runcam_uart.{c,h}, runcam_crc.{c,h}
- EV3635B/mCube_mc36xx_mcu_driver/MC36XX_MCU_3.1.0/src/drv (vendor MC36xx driver)
- cmake/pico_sdk_import.cmake, CMakeLists.txt

## Motion Classification
- Vendor MC36xx driver on I2C0 @ 100 kHz
- Sampling: ~54 Hz (CWAKE)
- Window: 30 samples; centered magnitude by de-meaning XYZ
- Peaks: local maxima above dynamic threshold; refractory spacing ~SAMPLE_RATE_HZ/4
- Frequency: peaks per window duration

Tune in lib/motion/motion_detect.c:
- walk_freq_min / walk_freq_max
- run_freq_min
- walk_amp_thr / run_amp_thr

USB logs include ACC (m/s^2), act + time, freq/amp/peaks/thr, hysteresis state and no-run seconds.

## Hysteresis (apps/hysteresis_recorder)
- Start: two consecutive 1-second windows with any RUN
- Stop: 3 consecutive seconds with no RUN samples (IDLE/WALK mix allowed)
- LED: ON during RUN; OFF otherwise
- USB-CDC enabled; waits briefly for enumeration

## RunCam Control (lib/runcam)
- Protocol header: 0xCC
- 5-key open sent 5 seconds after boot
- Start/Stop action: 0x01 (toggle) per device variant
- UART default: UART0 GP0/GP1 @ 115200

Change UART pins: edit runcam_uart_config_t in apps/hysteresis_recorder/main.c.

## Troubleshooting
- No USB logs: use a data cable; check /dev/tty.usbmodem* (macOS)
- Sensor init failed: verify wiring, pull-ups, 3V3/GND; keep 100 kHz; power-settle retries in place
- Camera not starting: ensure 5-key open; verify UART pins; 0x01 toggle is configured

## License
Add your license text here.
