# RDDRONE-BMS772
NuttX source code for RDDRONE-BMS772
## Disclaimer - CAUTION WARNING
### CAUTION - WARNING
Lithium and other batteries are dangerous and must be treated with care. 
Rechargeable Lithium Ion batteries are potentially hazardous and can present a serious FIRE HAZARD if damaged, defective or improperly used. Larger Lithium batteries and those used for industrial use involving high discharge current and frequent full discharge cycles require special precautions.

Do not connect this BMS to a lithium ion battery without expertise and  training in handling and use of batteries of this type.
Use appropriate test equipment and safety protocols during development. 

### Important Notice 
NXP provides the enclosed product(s) under the following conditions: 
This reference design is intended for use of ENGINEERING DEVELOPMENT OR EVALUATION PURPOSES ONLY. It is provided as a sample IC pre-soldered to a printed circuit board to make it easier to access inputs, outputs, and supply terminals. This reference design may be used with any development system or other source of I/O signals by simply connecting it to the host MCU or computer board via off-the-shelf cables. Final device in an application will be heavily dependent on proper printed circuit board layout and heat sinking design as well as attention to supply filtering, transient suppression, and I/O signal quality. 
The goods provided may not be complete in terms of required design, marketing, and or manufacturing related protective considerations, including product safety measures typically found in the end product incorporating the goods. 
Due to the open construction of the product, it is the user's responsibility to take any and all appropriate precautions with regard to electrostatic discharge. In order to minimize risks associated with the customers applications, adequate design and operating safeguards must be provided by the customer to minimize inherent or procedural hazards. For any safety concerns, contact NXP sales and technical support services. Should this reference design not meet the specifications indicated in the kit, it may be returned within 30 days from the date of delivery and will be replaced by a new kit. 
NXP reserves the right to make changes without further notice to any products herein. NXP makes no warranty, representation or guarantee regarding the suitability of its products for any particular purpose, nor does NXP assume any liability arising out of the application or use of any product or circuit, and specifically disclaims any and all liability, including without limitation consequential or incidental damages. 
Typical parameters can and do vary in different applications and actual performance may vary over time. All operating parameters, including Typical, must be validated for each customer application by customer’s technical experts. 
NXP does not convey any license under its patent rights nor the rights of others. NXP products are not designed, intended, or authorized for use as components in systems intended for surgical implant into the body, or other applications intended to support or sustain life, or for any other application in which the failure of the NXP product could create a situation where personal injury or death may occur. Should the Buyer purchase or use NXP products for any such unintended or unauthorized application, the Buyer shall indemnify and hold NXP and its officers, employees, subsidiaries, affiliates, and distributors harmless against all claims, costs, damages, and expenses, and reasonable attorney fees arising out of, directly or indirectly, any claim of personal injury or death associated with such unintended or unauthorized use, even if such claim alleges NXP was negligent regarding the design or manufacture of the part.

## sources
* To view the gitbook of this product see https://nxp.gitbook.io/rddrone-bms772/.
* To view the design files and the product on the NXP webpage see https://www.nxp.com/design/designs/rddrone-bms772-smart-battery-management-for-mobile-robotics:RDDRONE-BMS772. 

NXP has battery emulators that may be used during testing:
https://www.nxp.com/design/development-boards/analog-toolbox/6-cell-battery-pack-to-supply-mc33772-evbs:BATT-6EMULATOR.

This readme files will explain how to get the right nuttx and nuttx-apps repository with the BMS5.0 patches and build the BMS software (create a binary file).
This will work best on a linux machine, you could use a virtual machine for it.

See this webpage for the NuttX quickstart guide: https://nuttx.apache.org/docs/latest/quickstart/quickstart.html.

## Dependencies
git
unzip
curl

## Make sure git is installed
If git is not installed, open a terminal and type the following commands:
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```

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
(cd apps; git checkout nuttx-10.0.0 -b bms772)
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
(cd apps/nxp_bms/BMS_v1; git clone https://github.com/px4/public_regulated_data_types; cd public_regulated_data_types; git checkout 78b883891a582ff0eb41375433247b5ca0d44d21)
```
## Apply the patches
Apply the patch to the nuttx-apps.
```bash
(cd apps; patch -p1 < nxp_bms/BMS_v1/Patchfiles/0001-apps-patch-BMS5.0.patch)
```
Go the nuttx folder and checkout the right NuttX commit.
```bash
cd nuttx
```
```bash
git checkout 5c3ce49d8240a13899fb5a9d93b70c50140fcd41 -b bms772
```
Add the nuttx patch.
```bash
patch -p1 < ../apps/nxp_bms/BMS_v1/Patchfiles/0001-nuttx-patch-BMS5.0.patch
```
## Install the Kconfig tools and the crosscompiler if needed
When this is your first NuttX project, you need to install the Kconfig tools and the cross compiler. Otherwise you can skip this part and continue with "configure and make the binary".
### Install the Kconfig tools if needed
Install the build essentials and everything that is needed.
```bash
sudo apt-get install build-essential
```
```bash
sudo apt-get install flex
```
```bash
sudo apt-get install bison
```
```bash
sudo apt-get install gperf
```
```bash
sudo apt-get install libncurses5-dev
```
Go back from the nuttx directory and make a tools directory next to it.
```bash
cd ..
```
```bash
mkdir -p tools
```
```bash
cd tools
```
Clone the nuttx tools master.
```bash
git clone https://bitbucket.org/nuttx/tools/src/master
```
```bash
cd master
```
Apply the patch.
```bash
patch -p1 < ../../apps/nxp_bms/BMS_v1/Patchfiles/0001-aclocal-fix.patch
```
Configure and install it.
```bash
cd kconfig-frontends
```
```bash
./configure --enable-mconf --disable-nconf --disable-gconf --disable-qconf
```
```bash
make
```
```bash
sudo make install
```
```bash
sudo ldconfig
```
Go back to the nuttx directory
```bash
cd ../../../nuttx
```
### Install the cross compiler if needed 
If not already installed, install the cross compiler with the following command:
```bash
sudo apt install gcc-arm-none-eabi
```

## Configure and make the binary
Configure the BMS.
For normal use:
```bash
tools/configure.sh -e rddrone-bms772:bms
```
For debug purposes:
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
*   115200 Baud
*   8 data bits
*   1 stop bit

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
