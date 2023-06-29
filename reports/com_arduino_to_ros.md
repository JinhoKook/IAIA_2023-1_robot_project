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




#### com_arduino_to_ros.py

It is a code that reads the sensor value of Arduino and send data to ROS.

* **Code description**

  * Communication speed

  ```python
  def __init__(self):
          self.COMMU_HZ = 20 # If you want to change communication speed, you should change this value
          self.ser = serial.Serial("/dev/rfcomm0", 9600, timeout=1/self.COMMU_HZ)
          self.ser.flushInput()
          self.bridge          = CvBridge()
          self.msg_object_info = object_info()
          self.arduino_pub     = rospy.Publisher("arduno_to_ros", object_info, queue_size=10)
  ```

  

  * Flush

  ```python
  def run(self):
  
      rospy.init_node('arduino_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
      rate = rospy.Rate(5)                           # 루프 실행 주기 : 30hz
  
      while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
  
          if self.ser.inWaiting() > 0:
  
              recv_val = self.ser.readline().decode().rstrip()
  
              self.ser.flushInput() 
              # If there is no flush function, the data can be accumulated in communication packet.
              # Therefore, flush function can solve the communication delay problem.
  ```

* **Serial Module Download code**

```
sudo apt-get update
sudo apt-get install python3-serial
```
