#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리
import numpy as np
import math

from ur_python.msg import object_info, robot_state

class ImageProcessingNode:

    def __init__(self):

        self.bridge = CvBridge()

        self.flag_find = False
        self.flag_send_msg = False
        
        self.pub_object_info = rospy.Publisher("object_info_re", object_info, queue_size=10)
        
        self.msg_object_info = object_info()
        self.msg_robot_state = robot_state()

        self.detect_O = 1.0
        self.detect_X = 2.0

        self.cap_re = cv2.VideoCapture(2)

    def draw_cuboid(self, image, box):
        # Function to draw a cuboid shape on the given image
        pts = np.array(box, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(image, [pts], True, (0, 255, 0), 5)
        
        return image
    
    def calculate_aspect_ratio(self, box):
        # Function to calculate the aspect ratio of the given cuboid
        edge1 = np.linalg.norm(box[0] - box[1])
        edge2 = np.linalg.norm(box[1] - box[2])
        edge3 = np.linalg.norm(box[2] - box[3])
        edge4 = np.linalg.norm(box[3] - box[0])

        max_length = max(edge1, edge2, edge3, edge4)
        min_length = min(edge1, edge2, edge3, edge4)

        aspect_ratio = max_length / min_length

        return aspect_ratio

    def calculate_angle(self, box):
        # Function to calculate the angle between the longest edge of the given cuboid
        edge_lengths = [np.linalg.norm(box[i] - box[(i + 1) % 4]) for i in range(4)]
        longest_edge_index = np.argmax(edge_lengths)

        vertex1 = box[longest_edge_index][0]
        vertex2 = box[(longest_edge_index + 1) % 4][0]
        
        angle = math.atan2(vertex2[1] - vertex1[1], vertex2[0] - vertex1[0]) * 180 / math.pi
        angle -= 90  # Subtract 90 degrees to make the y-axis the reference for 0 degree
        
        if angle < -90:
            angle += 180
        
        elif angle > 90:
            angle -= 180
        
        return int(angle)

    def ImaegProcessing(self):

        self.cap_re.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_re.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
        min_contour_area = 500
        prev_angle = None

        while True:

            ret, frame      = self.cap_re.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            frame_original  = frame
            frame_original2 = cv2.cvtColor(np.zeros_like(frame_original), cv2.COLOR_BGR2GRAY)
            cv2.imshow('raw',frame_original)
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_gray = cv2.GaussianBlur(frame_gray, (9, 9), 10)
            
            _, frame_threshold = cv2.threshold(frame_gray, 100, 253, cv2.THRESH_BINARY)
            frame_canny = cv2.Canny(frame_threshold, 100, 1500)

            contours, _ = cv2.findContours(frame_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            lines_detected = False
        

            for contour in contours:
                contour_area = cv2.contourArea(contour)
                frame_original2 = cv2.drawContours(frame_original2,[contour], -1, (255,0,255),2)
            
            
                
                if contour_area < min_contour_area:
                    continue
            
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

                if len(approx) > 3:
                    
                    # self.draw_cuboid(frame, approx)
                    aspect_ratio = self.calculate_aspect_ratio(approx)
                    angle = self.calculate_angle(approx)

                    if aspect_ratio >= 1:
                        cv2.polylines(frame, [approx], True, (0, 0, 255), 1)
                    else:
                        cv2.polylines(frame, [approx], True, (255, 0, 0), 1)

                    if angle is not None:
                        
                        cv2.putText(frame, f"Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        print(f"Angle: {angle}")
                        prev_angle = angle
                        
                        self.msg_object_info.x = self.detect_O
                        self.msg_object_info.y = float(angle * math.pi / 180) # [rad]
                        
                        # Highlight the frame border with blue color
                        cv2.rectangle(frame, (0, 0), (frame.shape[1] - 1, frame.shape[0] - 1), (255, 0, 0), 2)

                        lines_detected = True
                    
                    else:
                        self.msg_object_info.x = self.detect_X
                        self.msg_object_info.y = 0.0
                        lines_detected = False

            cv2.imshow('draw',frame_original2)

            if not lines_detected:

                self.msg_object_info.x = self.detect_X
                self.msg_object_info.y = 0.0
                cv2.putText(frame, "Angle not detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            self.pub_object_info.publish(self.msg_object_info)
            self.flag_send_msg = True

            print(f"message sended(re): {self.msg_object_info.x:.2f}, {self.msg_object_info.y:.2f}")
            
            #cv2.imshow('canny', frame_canny)
            #cv2.imshow('frame_original', frame_original)

            cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
            cv2.imshow('frame', frame)

            cv2.namedWindow('For Extension 2', cv2.WINDOW_NORMAL)
            cv2.imshow('For Extension 2', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap_re.release()
        cv2.destroyAllWindows()
        
            
    def run(self):
        rospy.init_node('Image_Processing_RobotEye', anonymous=True) # 노드 초기화 및 이름 설정
        self.ImaegProcessing() 

if __name__ == '__main__':
    try:
        image_processing = ImageProcessingNode()    # ImageProcessingNode 클래스의 인스턴스 생성
        image_processing.run()                      # 노드 실행
        
    except rospy.ROSInterruptException:
        pass

