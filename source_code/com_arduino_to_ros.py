#!/usr/bin/env python3
import rospy
from ur_python.msg import object_info   
from cv_bridge import CvBridge, CvBridgeError   # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import serial

class ArduinoNode():
    def __init__(self):
        self.COMMU_HZ = 20
        self.ser = serial.Serial("/dev/rfcomm0", 9600, timeout=1/self.COMMU_HZ)
        self.ser.flushInput()
        self.bridge          = CvBridge()
        self.msg_object_info = object_info()
        self.arduino_pub     = rospy.Publisher("arduno_to_ros", object_info, queue_size=10)

    def run(self):

        rospy.init_node('arduino_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        rate = rospy.Rate(5)                           # 루프 실행 주기 : 30hz

        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
        
            if self.ser.inWaiting() > 0:

                recv_val = self.ser.readline().decode().rstrip()

                self.ser.flushInput()

                print('message received from arduino [mm]: ', recv_val)
                
                self.msg_object_info.x = float(recv_val) / 1000
                self.msg_object_info.y = float(recv_val) / 1000

                try:
                    self.arduino_pub.publish(self.msg_object_info)

                except CvBridgeError as e:
                    print(e)                            # CvBridge 변환 예외 처리
                
                rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기
            

def main():
    try:
        Ard2ROS = ArduinoNode()
        Ard2ROS.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()