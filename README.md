# RDDRONE-BMS772
NuttX source code for RDDRONE-BMS772

This readme files will explain how to get the right nuttx and nuttx-apps repository with the BMS3.4 patches and build the BMS software (create a binary file).
This will work best on a linux machine, you could use a virtual machine for it.

## Get the incubator nuttx and apps
Make a usefull folder to place the files in, like drones.
```bash
mkdir -p drones
```
```bash
cd drones
```

Clone the nuttx and nuttx apps git repositories.
```bash
git clone https://github.com/apache/incubator-nuttx.git nuttx
```
```bash
git clone https://github.com/apache/incubator-nuttx-apps.git apps
```
## Checkout the right commit
In the apps directory, checkout the right commit and branch.
```bash
(cd apps; git checkout 7a85bc75dcf5632154a09e75cfc431b6e25df463 -b bms772)
```
## Get the BMS in the nuttx-apps
Make a nxp_bms folder in the apps and the BMS_v1 folder in that folder
```bash
mkdir -p apps/nxp_bms
```
Clone this repository in that folder
```bash
(cd apps/nxp_bms; git clone https://github.com/NXPHoverGames/RDDRONE-BMS772.git BMS_v1)
```
Checkout the public regulated data types.
```bash
(cd apps/nxp_bms/BMS_v1; git clone https://github.com/PX4/public_regulated_data_types)
```
## Apply the patches
Apply the patch to the nuttx-apps.
```bash
(cd apps; patch -p1 < nxp_bms/BMS_v1/RDDRONE-BMS772/Patchfiles/0001-apps-patch-BMS3.4.patch)
```
Go the nuttx folder and checkout the right NuttX commit.
```bash
git checkout 1115f0104bd404454b72e9e02f4583febb0fe162 -b bms772
```
Add the nuttx patch.
```bash
patch -p1 < ../Patchfiles/0001-nuttx-patch-BMS3.4.patch
```
## configure and make the binary
Configure the BMS.
```bash
tools/configure.sh -e rddrone-bms772:bmsdebug
```
Make the binary with: 
```bash
make
```
## Using the command line interface (CLI) of the BMS
Make sure the BMS is powered and everything is connected properly to the BMS.

Use a UART terminal like minicom on a linux machine or PuTTY or teraTerm on a windows machine and connect to the right COM port.
The settings are:
*	115200 Baud
*	8 data bits
*	1 stop bit

## Programming the BMS with the JLink debugger
See the release notes of the BMS772 how to attach the debugger.

To program the BMS using a JLink debugger you need to have JLink installed.
Make sure the BMS is powered.

Open a terminal where the nuttx.bin file is located (probably in the nuttx folder).
Open JLink:
```bash
JLinkExe
```
Connect to it:
```bash
connect
```
Enter the correct device:
```bash
S32K144
```
Program it using SWD:
```bash
s
```
Use 1000kHz as target interface speed:
```bash
1000
```
Reset the device:
```bash
r
```
Reset the entire flash by sending these commands:
```bash
w1 0x40020007, 0x44     
```
```bash
w1 0x40020000, 0x80    
```
Load the nuttx binary at address 0
```bash
loadbin nuttx.bin 0
```
Reset the device
```bash
r
```
Run the program
```bash
g
```
Quit the JLinkExe 
```bash
q
```
