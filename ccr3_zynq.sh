#load the c++ lib

#set the defualt ip
#ifconfig eth0 192.168.137.250

#bring down the device
ip link set can0 down

#set the bitrate 500k
ip link set can0 type can bitrate 500000

#bring up the device
canconfig can0 start

#transmit and receive packets with standard id number
# cansend can0 -i 0x14 0x01

#checking link state
ip -d -s link show can0

#cheking net device state
# ifconfig can0

# cat /proc/net/can/stats

# excute the program
./ccr3_zynq/build/src/ccr3_zynq.elf

