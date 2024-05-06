#!/bin/bash

#build firmware
# Run your build command and capture the last line of its output
last_line=$(make | tail -n 1)

# Check the last line
if [ "$last_line" != "CP: nuttx.bin" ]; then
    echo "Error: Expected output not found!"
    exit 1
fi

#upload firmware
JLinkExe <<EOF
connect
S32K144
s
1000
r
w1 0x40020007, 0x44     
w1 0x40020000, 0x80  
connect  
loadbin nuttx.bin 0
r
g
q
EOF

# After JLinkExe commands are done, run the Python script
python3 /home/pi/NXP-BMS/BMS_programming_script.py


#connect to screen 
#screen /dev/ttyUSB0 115200
