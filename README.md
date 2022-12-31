[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![ROS: Melodic](https://img.shields.io/badge/ROS-Melodic-red.svg)](https://wiki.ros.org/melodic) [![web: auv.itu.edu.tr](https://img.shields.io/badge/web-auv.itu.edu.tr-blue.svg)](https://www.auv.itu.edu.tr) [![pipeline status](https://gitlab.com/itu-auv/electronics/mainboard-firmware/badges/master/pipeline.svg)](https://gitlab.com/itu-auv/electronics/mainboard-firmware/commits/master)
# Mainboard Firmware

## Overview

This repository is a source code of the firmware that is to be uploaded on Nucleo STM32F429ZI hardware. The STM32, is attached to a custum built board (PCB).

### License

The source code is released under [MIT License](LICENSE).

### Authors & Maintainers

**Authors:** 
- Sencer Yazici, [senceryazici@gmail.com](mailto:senceryazici@gmail.com)

**Maintainers:** 
- Sencer Yazici, [senceryazici@gmail.com](mailto:senceryazici@gmail.com)

### Working Environment
This repository consists packages of different platforms in a single folder. As a ros package, mainboard_firmware package is used with Ubuntu 18.04 and [ROS]() Melodic. As a PlatformIO development package, it is used and developed with latest PlatformIO in VSCode. In order to function as a package for both platforms, [platformio.ini](platformio.ini) and [package.xml](package.xml) files must be kept.


## Connection & Hardware
As a microcontroller ST Nucleo F429ZI is used.
![Nucleo STM32F429ZI](https://os.mbed.com/media/cache/platforms/Nucleo144_perf_logo_1024_qTjTDC0.jpg.250x250_q85.jpg)

#### Todo: 

- Pin Connection Table


    | Pin Number | Connection              | Description of Pin       |
    | ---------- | ----------------------- | ------------------------ |
    | PC2        | Battery Current Sensor  | ADC Pin                  |
    | PB1        | Battery Voltage Sensor  | ADC Pin                  |
    | PF8        | Motor 1                 | Motor 1 Signal Input Pin |
    | PD6        | Motor 2                 | Motor 2 Signal Input Pin |
    | PE5        | Motor 3                 | Motor 3 Signal Input Pin |
    | PE3        | Motor 4                 | Motor 4 Signal Input Pin |
    | PE2        | Motor 5                 | Motor 5 Signal Input Pin |
    | PG3        | Motor 6                 | Motor 6 Signal Input Pin |
    | PF0        | Motor 7                 | Motor 7 Signal Input Pin |
    | PD3        | Motor 8                 | Motor 8 Signal Input Pin |
    | PA0        | Motor 1 Current Sensor  | ADC Pin                  |
    | PC0        | Motor 2 Current Sensor  | ADC Pin                  |
    | PC0        | Motor 3 Current Sensor  | ADC Pin                  |
    | PC0        | Motor 4 Current Sensor  | ADC Pin                  |
    | PA3        | Motor 5 Current Sensor  | ADC Pin                  |
    | PA6        | Motor 6 Current Sensor  | ADC Pin                  |
    | PA7        | Motor 7 Current Sensor  | ADC Pin                  |
    | PB6        | Motor 8 Current Sensor  | ADC Pin                  |
    | PC10       | Light                   | Light Control Pin        |
    | PF13       | Kill Switch             | Kill Switch Pin          |
    | PC13       | User Button             | User Button Pin          |
    | PB0        | Green Led               | Led Pin                  |
    | PB7        | Blue Led                | Led Pin                  |
    | PB14       | Red Led                 | Led Pin                  |
    | PD2        | Xavier RX               | UART Receive Pin         |
    | PC12       | Xavier TX               | UART Transmit Pin        |
    | PE7        | Bottom Sonar RX         | UART Receive Pin         |
    | PE8        | Bottom Sonar TX         | UART Transmit Pin        |
    | PE10       | Doppler Velocity Log RX | UART Receive Pin         |
    | PE12       | Doppler Velocity Log TX | UART Transmit Pin        |

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [PlatformIO]() 
    Install VSCode (or supported editors), and install PlatformIO extension.
- [AutoPID](), [ping-arduino]()
    Provided under [lib/](lib/)

#### Building

To build from source, clone to repository to a catkin workspace, and use catkin build command

```sh
cd catkin_ws/src
git clone git@gitlab.com:itu-auv/electronics/mainboard-firmware.git
cd mainboard-firmware
git submodule update --init --recursive
cd ../../
catkin build 
```
This will build the ros package thus, you'll be able to use messages, launch files, configs etc. To upload the source code to the microcontroller, use PlatformIO's "Upload" button in VSCode, or your editor.

#### Setting Communication


- To find out the serial number of mainboard connection run below command in terminal
    - ```udevadm info -a -n /dev/tty<mainboard>```

- Note  ```ATTRS{idVendor}, ATTRS{idProduct}, ATTRS{serial}``` values.

- And under ```/etc/udev/rules.d``` Create a new file called ```auv.rules``` and put the following line in there
    - ```SUBSYSTEM=="tty", ATTRS{idVendor}=="<value>", ATTRS{idProduct}=="<value>", ATTRS{serial}=="<value>", SYMLINK+="ttySTM"```

For USB_CDC connection to the board, the rules line should be
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttySTM"
```

## Usage

After flashing the board, to establish the connection to the microcontroller:

```sh
roslaunch mainboard_firmware start_ubuntu.launch
```

## Config files

* **[default.yaml](config/default.yaml)** Contains the configuration parameters for Microcontroller, PID and Thruster Allocation Matrix is provided here.

## Launch files
The main launch files will be named as start_<PLATFORM>, where the platform may be OS X, Jetson, Ubuntu etc under [launch/](launch/) folder.

* **start_jetson.launch:** Starts the necessary nodes to connect to the microcontroller and other features.

* **start_osx.launch:** Starts the necessary nodes to connect to the microcontroller and other features.

## Nodes

### rosserial_python

Since microcontrollers using rosserial, aren't directly registered as node in ROS, a bridge node is implemented.
This node acts as a bridge between the node running in microcontroller and ROS.

#### Published Topics

* **`/turquoise/sensors/sonar_bottom`** ([std_msgs/Float32]())

	The range measurements computed from the bottom ping sonar ([ping-1d]()) from BlueRobotics.

* **`/turquoise/battery/state`** ([mainboard_firmware/BatteryState]())

	The real time voltage and current readings of the main battery, through APM.

* **`/turquoise/thrusters/current`** ([std_msgs/Float32MultiArray]())

	The current readings in amperes (A) computed from the current sensors located on board for each thruster.

#### Subscribed Topics
* **`/turquoise/cmd_vel`** ([geometry_msgs/Twist]())

	The velocity references for each axis, Linear X,Y,Z and Rotational X,Y,Z that are used to command the vehicle. A minimum frequency of 2.5 Hz must be provided in order to work without timeout (time > 400 ms). If messages are sent with time > 400 ms, the motors would stop after each reference.

* **`/turquoise/board/odom`** ([mainboard_firmware/OdometryMinimal]())

	The odometry of the vehicle, generated from [nav_msgs/Odometry]() message, excluding the covariances since it makes the communication unstable due to long message.
#### Macros


- [mainboard_conf.h](include/mainboard_firmware/mainboard_conf.h)

    | MACRO                                | DESCRIPTION                   |
    | ------------------------------------ | ----------------------------- |
    | MAINBOARD_KILLSWITCH_MODULE_ENABLED  | Enables corresponding module. |
    | MAINBOARD_SONAR_MODULE_ENABLED       | Enables corresponding module. |
    | MAINBOARD_TEMPERATURE_MODULE_ENABLED | Enables corresponding module. |
    | MAINBOARD_PRESSURE_MODULE_ENABLED    | Enables corresponding module. |
    | MAINBOARD_POWER_MODULE_ENABLED       | Enables corresponding module. |
    | MAINBOARD_SERIAL_DEBUG_ENABLED       | Enables corresponding module. |

#### Parameters

##### ROS Parameters
* **`allocation_cx`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) X.

* **`allocation_cy`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) Y.

* **`allocation_cz`** (FloatArray, default: "[1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) Z.

* **`allocation_rx`** (FloatArray, default: "[1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) x.

* **`allocation_ry`** (FloatArray, default: "[-1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) Y.

* **`allocation_rz`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) Z.


* **`pid_cx`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) X.

* **`pid_cy`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) Y.

* **`pid_cz`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) Z.

* **`pid_rx`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) x.

* **`pid_ry`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) Y.

* **`pid_rz`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) Z.

* **`pid_sat`** (FloatArray, default: "100.0")

	Saturation limit for PID's.

##### Firmware Parameters

- [params.h](include/mainboard_firmware/params.h)

    | PARAMETER             | DEFAULT   | TYPE     | DESCRIPTION                                |
    | --------------------- | --------- | -------- | ------------------------------------------ |
    | debug_baudrate        | 115200    | uint32_t | Debug interface baudrate (bit/s)           |
    | adc_resolution        | 12        | int      | ADC Resolution bits. Max 12.               |
    | adc_temp_avg_slope    | 2.5f      | float    | ADC temp slope, manual section 6.3.22      |
    | adc_temp_v25c         | 0.76f     | float    | ADC voltage reading at 25'C.               |
    | adc_vref_int          | 1210      | int      | ADC Vref internal                          |
    | adc_vbat_q            | 4.0f      | float    | ADC internal VBat ratio.                   |
    | acs712_mv_per_a       | 66.0f     | float    | ACS712 voltage to current ratio mv/A       |
    | acs712_offset         | 800.0f    | float    | ACS712 current offset in mA.               |
    | int sensor_interval   | 5U        | uint32_t | Passive sensor update interval in ms.      |
    | int apm_interval      | 20U       | uint32_t | APM update interval in ms                  |
    | apm_voltage_ratio     | 0.0085f   | float    | APM voltage ratio                          |
    | apm_current_ratio     | 0.013713f | float    | APM current ratio                          |
    | apm_low_batt_voltage  | 12.8f     | float    | Battery low voltage                        |
    | apm_min_batt_voltage  | 12.0f     | float    | Battery min voltage                        |
    | apm_max_batt_voltage  | 18.0f     | float    | Battery max voltage                        |
    | apm_max_batt_current  | 60.0f     | float    | Battery max current                        |
    | int ping_interval     | 50U       | uint32_t | Ping sonar ping interval in ms             |
    | ping_timeout          | 1000      | uint32_t | Ping sonar request timeout in ms           |
    | ping_baudrate         | 115200    | uint32_t | Ping sonar baudrate                        |
    | ms5837_fluid_density  | 997.0f    | float    | Fluid density, 1029 for seawater.          |
    | ms5837_interval       | 20        | uint32_t | MS5837 update interval in ms'              |
    | int tsys01_interval   | 10U       | uint32_t | TSYS01 temp sensor interval in ms          |
    | pid_loop_interval     | 12        | uint32_t | PID controller loop interval in ms         |
    | ros_publish_interval1 | 50        | uint32_t | ROS publish primary publish interval in ms |
    | ros_publish_interval2 | 500       | uint32_t | ROS secondary publish interval in ms       |
    | ros_spin_interval     | 1         | uint32_t | ROS spin interval in ms                    |
    | aux_len               | 1         | int      | Auxilary channel count                     |
    | aux_default_pulse     | 1100      | int      | Auxilary default pulse                     |
    | aux_min_pulse         | 1100      | int      | Auxilary min pulse                         |
    | aux_max_pulse         | 1900      | int      | Auxilary max pulse                         |
    | motor_default_pulse   | 1100      | int      | Motor default pulse                        |
    | motor_min_pulse       | 1100      | int      | Motor min pulse                            |
    | motor_max_pulse       | 1900      | int      | Motor max pulse                            |
    | motor_command_timeout | 400       | uint32_t | Motor command timeout in ms                |
    | motor_update_interval | 50        | uint32_t | Motor update interval in ms                |
    | startup_delay         | 2000      | uint32_t | delay for node to establish connection     |

#### Udev Rules for Communication
* Under /etc/udev/rules.d directory, ```auv.rules``` file. 

> SUBSYSTEM=="video4linux",ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9422", ATTRS{serial}=="SN0001", SYMLINK+="front_cam"
> SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttySTM"
> SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", SYMLINK+="ttySTLINK"
> SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="0017", SYMLINK+="ttyXSENS"

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://gitlab.com/itu-auv/electronics/issues).

## LICENSE
MIT, see [LICENSE](LICENSE) for details.