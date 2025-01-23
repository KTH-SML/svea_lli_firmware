# SVEA Low Level Interface (LLI)
This repository contains firmware for the SVEA Powerboard 2.
PlatformIO is used to build and upload the firmware. 
Instructions and scripts for installing PlatformIO 
together with other dependencies are included in this repository.
See the [Setup](#setup) section for further instructions. 


## Readme Structure

* [Repository structure](#Repository-structure)
	
	Structure of the repository

* [Setup](#Setup)
	
	Instructions for seting up the software required to flash the board.
	
* [Usage](#Usage)

	How to flash the board, and use the different targets.

* [Features](#Features)

	An overview of the features in the firmware.

* [ROS Related](#ROS-Related)

	Published and subscribed topics, and how they should be interpreted.

## Repository structure
- [Source code location](src/)

	This folder contains the source code for the firmware

- [Libraries](lib/)

	Contains libraries

- [Build scripts and other utilities](resources/)
- [PlatformIO configurations](/platformio.ini)
- [Developer instructions](docs/DEVELOPMENT.md)

## Setup

### New easiest way

1. **Download the firmware**  
   Get the latest `firmware.hex` from the [Releases](https://github.com/your-repo/releases).
2. **Get the loader**  
   Download the Teensy Loader application from [here](https://www.pjrc.com/teensy/loader.html).
3. **Connect the Teensy**  
   Plug the Teensy into your computer.
4. **Load the firmware**  
   Open the loader, click the document icon, and select the `firmware.hex` file.
5. **Upload the firmware**  
   Press the button on the Teensy while ensuring the "Auto" button in the loader is active (not greyed out).

---

There are three options for how to install platformIO.
The two first options will create a virtual environment for PlatformIO.
The last option will install the PlatformIO library directly with
your other python libraries.

### For development with Visual Code
_Recommended for development_
1. `git clone` this repository to a folder on your computer
2. [If a conda environemnt is acitve] Deactivate any active conda environment with `conda deactivate`
3. Install the platformIO plugin for VS Code as described in https://platformio.org/install/ide?install=vscode
4. Go to the root folder of this repository and run `bash init.sh`
5. Open the reository folder in VS Code

### Command line only
_Recommended for just flashing new firmware_
1. `git clone` this repository to a folder on your computer
2. [If a conda environemnt is acitve] Deactivate any active conda environment with `conda deactivate`
3. Install PlatformIO by executing: `python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"`
4. Go to the root folder of this repository and run `bash init.sh`

### I'm in a hurry and I just need to flash this Powerboard NOW

```
git clone https://github.com/kth-sml/svea_lli_firmware
cd svea_lli_firmware
python3 -m venv venv
. venv/bin/activate
pip install -r requirements.txt
./init.sh
pio run -t upload
```

## Usage
### Compile and upload firmware
#### From command line
1. Navigate to the root folder of the repository
2. Actiate the PlatformIO vritual environment with `source ~/.platformio/penv/bin/activate`
3. Connect the Powerboard via USB
4. Execute `pio run -t upload`

#### From VS Code
VS Code can be used to build and upload the firmware to the Powerboard if the PlatformIO plugin is installed. 
1. Open the root folder of this repository in VS Code
2. Go to the _PlatformIO_ tab in VS code
3. Klick on the _Default_ folder
4. Open the _General_ tab
5. Klick on _Upload All_ (Only one environemnt will actually be built and uploaded) 
The virtual environment will automatically be activated by the PlatformIO plugin.

### Available environments 
There are 5 different environemnts defined in [platformio.ini](platformio.ini). 
The difference between these environemnts is how the rosserial libraries are created.
The environment can be set with the `-e` flag, e. g. `-e *environment name*`.

#### use_included_libs
The default environement. 
Uses the rosserial libraries included in [lib/ros_included](lib/ros_included). 
This is the environment that runs if no 
`--environment` or `-e` argument is passed to `pio run`. 
This environment is can be used if no messages defintions have been added or changed.
Note that most of the standard message defitions,
like `std_msgs`, `geometry_msgs`, and `nav_msgs` are already included.

#### build_local_libs
Build the rosserial libraries from the currently sourced ROS workspaces.
Rosserial and all required message definitions have to be included
in these workspaces, or the compilation will fail.
Intended to be used for development where messages have been added
or altered on the current machine.
The newly built rosserial libraries will be placed in a folder named
`lib/ros_local`.
__Note: This environment generates rosserial messages, but does not build the workspaces.__
__The workspaces will have to be built manually for any changes to message definitions to take effect.__

#### use_local_libs
Same as `use_included_libs` but will use
rosserial libraries located in [lib/ros_local](lib/ros_local) 
(previously generated by `build_local_libs`), instead of [lib/ros_included](lib/ros_included).

#### download_and_build_remote_libs
Download both rosserial and all messages from the `vehicle_msgs` repository, 
build them and place the rosserial library in [lib/ros_remote](lib/ros_remote).
Some temporary ROS workspaces will be placed in a _tmp_ folder in the root of the repository.

#### use_remote_libs
Same as `use_included_libs` but will use
rosserial libraries located in [lib/ros_remote](lib/ros_remote) instead
(previously generated by `download_and_build_remote_libs`), instead of [lib/ros_included](lib/ros_included).


## Features
This is a summary of the features that are implemented in the firmware.
Some features that are not implemnted, but is supported by the hardware
are also listed under [Not Implemented](#Not-Implemented).

* **PWM reading**

	Up to five pwm signals from a receiver can be read and translated to acuation values. 

* **PWM actuation**

	PWM signals can be sent to steering, velocity, gear, and differential locks.

* **ROS connection**

	The firmware can communicate with a ROS network over rosserial.	
	See the [ROS Related](#ROS-Related) section for details.

* **On board LEDs**

	The four on board LEDs can be controlled by the Teensy.
	Currently the colors represent: 

	0. Green/Red: ROS is active/idle
	1. Green/Red: The remote is connected/disconnected
	2. Green/Red: Remote override is on/off
	3. Green/Red: Emergency break on/off

	If the remote is disconnected and ROS is inactive
	the LED will go into disco mode.


* **Servo Power off on idle**

	Power to the servos will be cut if the remote is disconnected,
	and nothing is being sent through ROS.

* **Emergency Breaking**

	An emergency break can be initiated trough ROS.
	All other actuation will be ignored until the emergency is cleared. 

* **Steering calibration**

	The steering can be calibrated with the remote.
	Initiate callibration by holding down button 0 for 1 second.
	The LEDs should turn yellow. Now turn the tires as far to the left
	as they can go without pushing against the chassis. 
	Push button 0 again. The LEDs should turn blue. 
	Turn the tire as far to the right as they can go without
	pushing against the chassis. 
	Push button 0 again and the LEDs should blink for a short while.
	The callibration is complet and the values have been saved to flash.


### Not Implemented

* **External LEDs**

	External LED connectors are available on the SPMB2,
	however these are currently not supported by the firmware.

* **Power measurements through INA260**

	There is a INA260 power sensor on the SPMB2 that 
	can be used to measure the power consumed by the powerboard
	(excluding servos). The INA260 is connected to I2C1 on the Teensy.


## ROS Related

The following topics are subscribed to and published on by the firmware.

#### Subscribed Topics

* **`lli/actuation_request`** ([svea_msgs/lli_actuation.msg])

	Takes request for an actuation to be set.
	
	If nothing is published for `SW_TIMEOUT` milli seconds,
	the ROS actuation will be considered idle.
	If the remote is not activem the acuation will go idle,
	otherwise control will be handed over to the remote.

	Values of `-128` are treated as do not change i.e. whichever value 
	that was previously actuated will continue to be actuated.

	The *ctrl* field has no effect except that it can be used to trigger 
	a response on `lli/actuated` by changing the flags instead of the actuation values.
 

* **`lli/emergency`** ([svea_msgs/lli_emergency.msg])

	Emergency brake which will override other actuation requests. 
	If the value of the message is *True*, the vehicle will break
	and ignore any further input until either:
	
	- A message with *False* is sent
	- A time defined by `EMERGENCY_T_CLEAR_LIMIT` has passed and the remote is in 
	the override position.


#### Published Topics

* **`lli/actuated`** ([svea_msgs/lli_actuation.msg])

	The currently actuated values will be published here, independent of source.
	Values will only be published if there is a change. 
	No new message will be published if an actuation value is received that is identical 
	(same stering, velocity, gear, differential locks, and control flags) 
	to the one already being actuated.

	The first four bits in the *ctrl* value represents
	0. ROS is considered idle
	1. The remote is considered idle
	2. The remote override is turned on
	3. An emergency has been set 

* **`lli/remote`** ([svea_msgs/lli_actuation.msg])

	Readings from the remote. The control flags are the same as for `lli/actuated`.

* **`lli/encoder`** ([svea_msgs/lli_encoder.msg])

	Encoder tics and time intervals for each wheel if encoders are mounted.
	If no encoders are mounted, zeero readings will still be published.


## FAQ

<details>
<summary>
The ROS header file cannot be found.
</summary>


Ensure that you ran `init.sh` (required to use automated hooks) and that you restarted your login session. (reboot or re-login)
You can test if `echo ${tmw_DIR}` outputs the directory of your `teensy_firmware` clone.

Also ensure that you have a `catkin` compatible Python version. (Note that Python 3.7 has a trollius and async issue. Easiest is to switch your virtualenvironment to Python2 and install the dependencies with `pip install -r requirements.txt`. Also ensure that after the virtual environment switch you should execute `source /opt/ros/${ROS_DISTRO}/setup.{sh,zsh}`).
</details>


<details>
<summary>
The build fails and throws some error.
</summary>


Inspect the logs generated by the hooks under `./firmware/log/middleware.log`


The output could look similar to the following snippet.
```bash
(vp2) firmware cat log/middleware.log
Input arguments received:
	 template git clone https://github.com/prothen/testbed_msgs.git
Set project name: template
Parse repository:
	 git clone https://github.com/prothen/testbed_msgs.git
Received additional libraries.
ROS1 interface chosen. Configuring dependencies ... (tzz its 2021...)
BUILD rosserial arduino from upstream:
remote: Enumerating objects: 467, done.
remote: Counting objects: 100% (467/467), done.
remote: Compressing objects: 100% (351/351), done.
remote: Total 467 (delta 115), reused 297 (delta 60), pack-reused 0
Receiving objects: 100% (467/467), 299.88 KiB | 3.57 MiB/s, done.
Resolving deltas: 100% (115/115), done.
----------------------------------------------------------------
```
</details>

## Contribution
Any contribution is welcome.
If you find missing instructions or something did not work as expected please create an issue and let me know.

## License
See the `LICENSE` file for details of the available open source licensing.
