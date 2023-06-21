#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy

from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능

import cv2                          # OpenCV 라이브러리
import numpy as np
import math

from ur_python.msg import object_info, robot_state

class ImageProcessingNodeMS:

    def __init__(self):

        self.bridge = CvBridge()

        self.flag_find = False
        self.flag_send_msg = False

        self.pub_object_info = rospy.Publisher("object_info_ms", object_info, queue_size=10)
        
        self.msg_object_info = object_info()
        self.msg_robot_state = robot_state()

        self.cx_prev = 0
        self.cy_prev = 0

        self.cx = 0
        self.cy = 0
        self.radi = 0

        self.lower_bound = np.array([0, 192, 37])
        self.upper_bound = np.array([66, 255, 193])

        self.cap_ms = cv2.VideoCapture(4)

    def ImaegProcessing(self):

        self.cap_ms.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_ms.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        trackbarName = "Trackbar Windows: RED"
        cv2.namedWindow(trackbarName)

        # Create trackbars for lower and upper bound values
        cv2.createTrackbar("H_min", trackbarName, 0, 179, lambda x : x)
        cv2.createTrackbar("S_min", trackbarName, 0, 255, lambda x : x)
        cv2.createTrackbar("V_min", trackbarName, 0, 255, lambda x : x)

        cv2.createTrackbar("H_max", trackbarName, 0, 179, lambda x : x)
        cv2.createTrackbar("S_max", trackbarName, 0, 255, lambda x : x)
        cv2.createTrackbar("V_max", trackbarName, 0, 255, lambda x : x)
        
        cv2.setTrackbarPos("H_min", trackbarName, self.lower_bound[0])
        cv2.setTrackbarPos("S_min", trackbarName, self.lower_bound[1])
        cv2.setTrackbarPos("V_min", trackbarName, self.lower_bound[2])

        cv2.setTrackbarPos("H_max", trackbarName, self.upper_bound[0])
        cv2.setTrackbarPos("S_max", trackbarName, self.upper_bound[1])
        cv2.setTrackbarPos("V_max", trackbarName, self.upper_bound[2])

        while True:

            h_min = cv2.getTrackbarPos("H_min", trackbarName)
            s_min = cv2.getTrackbarPos("S_min", trackbarName)
            v_min = cv2.getTrackbarPos("V_min", trackbarName)
            h_max = cv2.getTrackbarPos("H_max", trackbarName)
            s_max = cv2.getTrackbarPos("S_max", trackbarName)
            v_max = cv2.getTrackbarPos("V_max", trackbarName)

            self.lower_bound = np.array([h_min, s_min, v_min])
            self.upper_bound = np.array([h_max, s_max, v_max])

            ret, image = self.cap_ms.read()
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
         
            mask = cv2.inRange(hsv_image, self.lower_bound, self.upper_bound)
            result = cv2.bitwise_and(image, image, mask=mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:

                if cv2.contourArea(contour) > 200:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    self.flag_find = True

                    self.cx = ((x + int(w / 2)) - 160)/700 + 0.13
                    self.cy = (240 - (y + int(h / 2)))/500 + 0.49 

                    self.radi = math.sqrt(self.cx ** 2 + self.cy ** 2)  

                    if self.radi > 0.82: 
                        self.cx = self.cx_prev
                        self.cy = self.cy_prev
                    
                    self.cx_prev = self.cx
                    self.cy_prev = self.cy
                else:
                    self.flag_find = False
                    self.cx = self.cx_prev
                    self.cy = self.cy_prev

            
            self.convert_img2real()
            self.pub_object_info.publish(self.msg_object_info)
            
            
            self.flag_send_msg = True
            print(f"message sended(ms:: x, y): {self.msg_object_info.x:.3f}, {self.msg_object_info.y:.3f}")
            print(f"message sended(ms:: dist): {self.radi:.2f}")

            image = cv2.rotate(image, cv2.ROTATE_180) # 180도 회전
            cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
            cv2.imshow("image", image)

            cv2.namedWindow("For Extension", cv2.WINDOW_NORMAL)
            cv2.imshow("For Extension", image)

            # cv2.imshow("mask", mask)

            if cv2.waitKey(1) == ord('q'): # 1ms 동안 키보드 입력 대기 
                break                  


    def convert_img2real(self):

        self.msg_object_info.x = self.cx
        self.msg_object_info.y = self.cy

    def recv_robot_state(self, data):
        self.msg_robot_state.move = data.move
        if self.msg_robot_state.move == 0:
            self.flag_send_msg = False
        
            
    def run(self):
        rospy.init_node('Image_Processing', anonymous=True) # 노드 초기화 및 이름 설정
        self.ImaegProcessing() 

if __name__ == '__main__':
    try:
        image_processing = ImageProcessingNodeMS()    # ImageProcessingNode 클래스의 인스턴스 생성
        image_processing.run()                      # 노드 실행
        
    except rospy.ROSInterruptException:
        pass