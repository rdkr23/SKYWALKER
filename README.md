# P4-skywalker
Guide for how to connect, run and operate the walking manipulator.

There are a few steps before the program can be run. Clone the repository and install the dependencies.

## Configure your internet for auto syncronization

Install chrony
```
sudo apt update
sudo apt install chrony
```
Edit the config:
```
sudo nano /etc/chrony/chrony.conf
```

Add these two lines at the bottom:
  
```
allow 192.168.0.0/16SS
local stratum 10      
```
If not already done, connect the computer to the wired network.

Configure the network to have an address of 192.168.1.196, netmask of 255.255.255.0 and a gateway of 192.168.1.1

## Run the program
Restart the system by disconnecting and connecting the power. Wait around 2 minutes so the raspberry PIs can start up. Afterwards run the program with:
```
ROS2 launch p4_launcher p4_main.launch.py
```
