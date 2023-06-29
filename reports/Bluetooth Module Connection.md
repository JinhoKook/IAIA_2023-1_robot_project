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



#### Bluetooth Module Connection

* **Check bluetooth device address**

![블루투스모듈연결사진](https://github.com/Ohjeahyun1/EC-jeahyun-447/assets/113822586/a081c67b-d589-4ea8-a431-218ce5916f1a)

* **Bluetooth device connection code**

```
sudo rfcomm connect <number> <address>
# In my case, sudo rfcomm connect 0 98:DA:60:05:2A:8A
```

* **Serial Module Download code**

```
sudo apt-get update
sudo apt-get install python3-serial
```

