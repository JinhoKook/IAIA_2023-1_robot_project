---
Handong Global University

School of Mechanical and Control Engineering

2023-1 Industrial AI & Automation

* 21400685 | Seok-Won Jung

* 21700384 | Tae-Woong Song

* 21800031 | Jin-Ho Kook

* 21800447 | Jae-Hyun Oh
---



# **Robot System for Motion Tracking and Precise Parsing in Disaster Sites**



## Environment Setting

#### (1) Bluetooth setting

##### **Permission setting**

* The computer name must register under the following command. (user) is a computer name.

```
sudo usermod -a -G dialout $(USER)   
```



* reboot code

```
sudo reboot now
```



* Depending on USB recognition, the code should be different depending on where the port is connected.

```
sudo chmod a+rw /dev/rfcomm0
```



* When using Raspberry Pi's pinheader (/dev/tyAMA0) for serial communication

```
sudo usermod -a -G tty $USER
sudo chmod g+r/dev/ttyAMA0
```



##### **tty checking  method**

* Check serial number

```
dmesg | grep tty
```



* Check serial port settings status

```
stty -F [devicename]
```



#### **(2) Serial Module Download**

```
sudo apt-get update
sudo apt-get install python3-serial
```
