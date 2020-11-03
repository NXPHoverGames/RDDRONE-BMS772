# RDDRONE-BMS772
NuttX source code for RDDRONE-BMS772

This readme files will explain how to get the right nuttx and nuttx-apps repository with the BMS3.4 patches and build the BMS software (create a binary file).
This will work best on a linux machine, you could use a virtual machine for it.

Get the files in this repository. 

## Get the incubator nuttx and apps
Make a usefull folder to place the files in, like drones or keep it in the folder RDDRONE-BMS772.
Linux command:
```bash
mkdir -p drones
```
```bash
cd drones
```

Clone the nuttx and nuttx apps git repositories.
For nuttx clone:
https://github.com/apache/incubator-nuttx.git
Git command:
```bash
git clone https://github.com/apache/incubator-nuttx.git
```
For nuttx-apps clone:
https://github.com/apache/incubator-nuttx-apps.git
Git command:
```bash
git clone https://github.com/apache/incubator-nuttx-apps.git
```
Make symbolic links for the nuttx and the apps.
Linux command:
```bash
ln -s incubator-nuttx nuttx
```
```bash
ln -s incubator-nuttx-apps apps
```
## Checkout the right commit
In the apps directory, checkout the right commit and branch.
Git hash: 7a85bc75dcf5632154a09e75cfc431b6e25df463
Linux command:
```bash
(cd apps; git checkout 7a85bc75dcf5632154a09e75cfc431b6e25df463 -b bms772)
```
## Get the BMS in the nuttx-apps
Make a nxp_bms folder in the apps.
Linnux command:
```bash
mkdir -p apps/nxp_bms
```
Make the BMS_v1 folder in the nxp_bms folder.
Linnux command:
```bash
mkdir -p apps/nxp_bms/BMS_v1
```
Copy the scr and inc folder in the BMS_v1 folder.
cp -r folder_directory_src apps/nxp_bms/BMS_v1
cp -r folder_directory_inc apps/nxp_bms/BMS_v1
Example:
```bash
cp -r src apps/nxp_bms/BMS_v1
```
```bash
cp -r inc apps/nxp_bms/BMS_v1
```
Copy the Makefile, the Make.defs and the Kconfig to the BMS_v1. Linux command:
```bash
cp {Makefile,Make.defs, Kconfig} apps/nxp_bms/BMS_v1
```
Checkout the public regulated data types.
https://github.com/PX4/public_regulated_data_types
Linux command:
```bash
(cd apps/nxp_bms/BMS_v1; git clone https://github.com/PX4/public_regulated_data_types)
```
## Apply the patches
Apply the patch to the nuttx-apps.
patch -p1 < ../Patchfiles/0001-apps-patch-BMS3.4.patch
Linux command:
```bash
(cd apps; patch -p1 < ../Patchfiles/0001-apps-patch-BMS3.4.patch)
```
Go the nuttx folder and checkout the right NuttX commit.
Git command:
git checkout 1115f0104bd404454b72e9e02f4583febb0fe162 -b bms772
Linux command:
```bash
cd nuttx
```
```bash
git checkout 1115f0104bd404454b72e9e02f4583febb0fe162 -b bms772
```
Add the nuttx patch.
patch -p1 < ../Patchfiles/0001-nuttx-patch-BMS3.4.patch
Linux command:
```bash
patch -p1 < ../Patchfiles/0001-nuttx-patch-BMS3.4.patch
```
## configure and make the binary
Configure the BMS.
Either use:
```bash
tools/configure.sh -e rddrone-bms772:bmsdebug
```
Or 
```bash
tools/configure.sh -e rddrone-bms772
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
