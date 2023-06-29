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



#### **Arduino Code for Data Acquisition**

* **Bluetooth communication module download**

  1) Open Arduino IDE   
  2) Tools -> Library management
  3) Install ***Adafruit_VL53L0X***

  

* **Code description**
  * Current bluetooth pins: 10(TX), 11(RX)
  * Communication speed: about 100Hz

```c
SoftwareSerial BTSerial(10, 11); # Bluetooth communication pin number setting

...

void loop() {

	...

	delay(10); # Communication speed can be adjusted by changing the delay value
	
}
```



* **Connect Arduino to computer and embed code through Arduino IDE**


