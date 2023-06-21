#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy

from tf.transformations import *
from math               import tau

from move_group_python_interface    import MoveGroupPythonInterface
from ur_python.msg                  import object_info, robot_state
from pynput.mouse                   import Listener
from geometry_msgs.msg import Quaternion

import cv2
import time
import copy
import numpy as np

class UR5e_Move_With_Camera():

    def __init__(self):

        self.MODE1 = 0
        self.MODE2 = 1
        self.i = 0
        
        self.MODE =  self.MODE1
        
        self.DET_BARRIER_O = 1.0
        self.DET_BARRIER_X = 2.0

        self.ur5e = MoveGroupPythonInterface()
        
        self.cur_pose = [0.0, 0.0, 0.0]
        self.cur_rpy = [0.0, 0.0, 0.0]
        self.target_pose = [0.0, 0.0, 0.0]

        # self.ur5e = MoveGroupPythonInterface()

        self.msg_object_info_ms = object_info() # RECEIVE FROM MOTION STAGE
        self.msg_object_info_re = object_info() # RECEIVE FROM ROBOT EYE

        # Receive message from camera 
        self.sub_object_info_ms     = rospy.Subscriber("object_info_ms", object_info, self.detection_callback_ms)
        self.sub_object_info_re     = rospy.Subscriber("object_info_re", object_info, self.detection_callback_re)
        self.sub_object_info_ard    = rospy.Subscriber("arduno_to_ros", object_info, self.detection_callback_ard2ros)
        
        # Mode Change Flag
        self.mode_change = 0
        
        self.flag_recv_msg_ms = False
        self.flag_recv_msg_re = False
        self.flag_recv_msg_ard2ros = False
        
        self.pre_cmd_x = 0.13
        self.pre_cmd_y = 0.49

        self.cmd_x_ms = 0.0
        self.cmd_y_ms = 0.0
        
        self.state_recog_barrier = 0.0
        self.cmd_angle_end_effector = 0.0
        
        self.sensor_dist1 = 0.0
        self.sensor_dist2 = 0.0

        print("INITIALIZED..!")


    def print_msg(self, msg):
        print('\t' + msg)
        print('\n\n')


    def print_val(self, msg, val):
        print('\t' + msg + ': ', val)
        print('\n\n')


    def print_line(self):
        print('\t' + '=======================')
        print('\n\n')


    def print_title(self, msg):
        print('\t' + '=======================')
        print('\n\n')
        print(msg)
        print('\n\n')
        print('\t' + '=======================')
        print('\n\n')


    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_FLAG_L_BUTTON:
            self.MODE = self.MODE2
            print("MODE CHANGED..!")


    def detection_callback_ms(self, data):
        self.flag_recv_msg_ms = True
        self.cmd_x_ms = data.x
        self.cmd_y_ms = data.y
        

    def detection_callback_re(self, data):
        self.flag_recv_msg_re = True
        self.state_recog_barrier = data.x       # state_recog_barrier == 1.0 --> barrier detected / state_recog_barrier == 2.0 --> barrier not detected
        self.cmd_angle_end_effector = data.y    # 


    def detection_callback_ard2ros(self, data):
        self.flag_recv_msg_ard2ros = True
        self.sensor_dist1 = data.x
        self.sensor_dist2 = data.y


    def get_cur_pose_rpy(self, robot):
        cur_pose = robot.manipulator.get_current_pose().pose
        cur_quat = [cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w]
        cur_rpy  = euler_from_quaternion(cur_quat)   # current orientation: Quat -> Euler
        cur_pose = [cur_pose.position.x, cur_pose.position.y, cur_pose.position.z] 
        return cur_pose, cur_rpy


    def get_cur_joint_val(self, robot):
        current_joint = robot.manipulator.get_current_joint_values()
        target_joint = copy.deepcopy(current_joint)
        return target_joint
    

    def check_mode(self):
        if self.mode_change == 1 and self.state_recog_barrier == self.DET_BARRIER_O:
            self.MODE = self.MODE2
            self.print_title('MODE 2')


    def mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_FLAG_LBUTTON:    
            self.mode_change = 1
            print("MODE CHANGE FLAG ON..!")


    def initialization(self):

        input("PRESS ENTER TO GO INITIAL POSITION..!")
        self.ur5e.move_to_standby()
        input("PRESS ENTER TO BREAK INITIAL POSITION..!")

        self.cur_pose, self.cur_rpy = self.get_cur_pose_rpy(self.ur5e)
        self.target_pose = self.cur_pose

        self.target_pose[2] = self.cur_pose[2] - 0.1
        self.ur5e.go_to_pose_abs(self.target_pose, self.cur_rpy)

        print("ROBOT INITIAL POSITION MOVE COMPLETE..!")
        input("PRESS ENTER TO GO MODE 1..!")
        
    def get_planning_time_ur5e(self, absolute_xyz, absolute_rpy):
        
        current_pose = self.ur5e.manipulator.get_current_pose().pose

        # target position
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x = absolute_xyz[0]
        target_pose.position.y = absolute_xyz[1]
        target_pose.position.z = absolute_xyz[2]

        # Transform target_rpy to target_quat
        target_quat = quaternion_from_euler(absolute_rpy[0], absolute_rpy[1], absolute_rpy[2])

        # Apply target_quat to the orientation of target_pose
        target_pose.orientation = Quaternion(target_quat[0], target_quat[1], target_quat[2], target_quat[3])

        self.ur5e.manipulator.set_pose_target(target_pose)
        plannig_time = self.ur5e.manipulator.get_planning_time()

        self.print_val("Expected planning time", plannig_time)

    def mode1_control(self):

        if self.cmd_y_ms < 0.49: 
            
            self.cmd_y_ms = 0.49
        
        if abs(self.cmd_x_ms - self.pre_cmd_x) > 0.005 or abs(self.cmd_y_ms - self.pre_cmd_y) > 0.005:

            target_pose_abs_xyz = [self.cmd_x_ms, self.cmd_y_ms, self.cur_pose[2]] 

            # self.get_planning_time_ur5e(target_pose_abs_xyz, self.cur_rpy)

            start_time = rospy.Time.now().to_sec()
            self.ur5e.go_to_pose_abs(target_pose_abs_xyz , self.cur_rpy)
            end_time = rospy.Time.now().to_sec()
            take_time = end_time - start_time
            self.print_val("Real planning time", take_time)

            self.pre_cmd_x = self.cmd_x_ms
            self.pre_cmd_y = self.cmd_y_ms

            self.flag_recv_msg_ms = False
            self.flag_recv_msg_re = False


    def rotate_end_effector(self):

        while(abs(self.cmd_angle_end_effector) > 0.1):

            cur_joint = self.get_cur_joint_val(self.ur5e)
            angle_end_effector_scaled = 0.0
            
            if self.cmd_angle_end_effector < 0:
                print(f"end_effctor_mi: {self.cmd_angle_end_effector:.2f}")
                cur_joint[5] = 1.13 * (cur_joint[5] + self.cmd_angle_end_effector)
                
            else:
                print(f"end_effctor_plus: {self.cmd_angle_end_effector:.2f}")
                cur_joint[5] = cur_joint[5] + self.cmd_angle_end_effector

            print(f"end: {cur_joint[5]:.2f}")

            self.ur5e.go_to_joint_state(cur_joint)
            cv2.waitKey(500)


    def go_down_to_barrier(self):

        # 거리 값만큼 내려가게 하는 코드(값 수정 필요)
        cur_pose, cur_rpy = self.get_cur_pose_rpy(self.ur5e)
        target_pose = cur_pose
        cv2.waitKey(1500)

        target_pose[2] = cur_pose[2] - self.sensor_dist1 + 0.12

        self.print_msg('Going down to barrier...')
        self.ur5e.go_to_pose_abs(target_pose, cur_rpy)
        self.print_msg('Arrived to barrier..!')
    

    def grip_on_barrier(self):

        self.print_msg('Gripping...')
        cv2.waitKey(2000)
        self.ur5e.grip_on()
        self.print_msg('Gripped..!')


    def go_back_to_standby_pos(self):
        # 물체를 잡으면 초기좌표로 이동
        self.print_msg('Going bask to inital position...')
        self.ur5e.move_to_standby()
        self.print_msg('Initial position arrived..!')


    def go_to_garbage_dump(self):
        
        # 책상 뒤로 돌아가도록 만드는 코드
        self.print_msg('Carrying barrier to garbage dump...')

        cur_joint = self.get_cur_joint_val(self.ur5e)
        target_joint = cur_joint
        target_joint[0] = target_joint[0] - tau / 2
        self.ur5e.go_to_joint_state(target_joint)

        self.print_msg('Arrived garbage dump above..!')


    def go_down_garbage_dump(self):
        # 내려놓기 위한 거리를 측정 및 내려가는 코드    
        self.print_msg('Putting barrier to garbage dump...')
        cv2.waitKey(1000)
        cur_pose, self.cur_rpy = self.get_cur_pose_rpy(self.ur5e)
        self.target_pose = cur_pose
        if self.i == 0:
            self.target_pose[0] = cur_pose[0] + 0.15
        elif self.i == 1:
            self.target_pose[0] = cur_pose[0] - 0.05
        elif self.i == 2:
            self.target_pose[0] = cur_pose[0] - 0.22
        self.ur5e.go_to_pose_abs(self.target_pose, self.cur_rpy)
        cv2.waitKey(1500)
        cur_pose, self.cur_rpy = self.get_cur_pose_rpy(self.ur5e)
        self.target_pose = cur_pose
        self.target_pose[2] = cur_pose[2] - 0.27
        self.ur5e.go_to_pose_abs(self.target_pose, self.cur_rpy)

        self.print_msg('Arrived to garbage dump near...')

    
    def grip_off_barrier(self):
        # 물체 내려 놓기
        cv2.waitKey(2000)
        self.ur5e.grip_off()
        self.print_msg('Barrier be put..!')


    def go_to_prev_pos(self):
        # 제자리에서 Z 값만 올리기 위한 코드
        self.print_msg('Going back to initial position...')
        self.target_pose[2] = self.target_pose[2] + 0.27
        self.ur5e.go_to_pose_abs(self.target_pose, self.cur_rpy)
        cv2.waitKey(2000)
    

    def check_mode_change_state(self):
        # 이동 한 곳의 좌표 확인
        self.cur_pose, _ = self.get_cur_pose_rpy(self.ur5e) 
        
        # 로봇 좌표와 트레킹 오브젝트 좌표가 초기 위치라면 Mode 1으로 이동
        self.print_msg('Setting TRACKING OBJECT to x == 0.13 & y == 0.49..')

        while True:
            x = abs(self.cur_pose[0] - self.cmd_x_ms)
            y = abs(self.cur_pose[1] - self.cmd_y_ms)
            if x < 0.01 and y < 0.01:
                break

        self.cur_pose, self.cur_rpy = self.get_cur_pose_rpy(self.ur5e)
        self.target_pose = self.cur_pose
        self.target_pose[2] = self.cur_pose[2] - 0.1
        self.ur5e.go_to_pose_abs(self.target_pose, self.cur_rpy)
        
        self.print_msg('TRACKING OBJECT be set..!')
        
        self.i += 1
        if self.i > 2:
            self.i = 0
        self.MODE = self.MODE1
        self.print_title('MODE 1')
        self.mode_change = 0

        self.flag_recv_msg_re = False  
        self.flag_recv_msg_ard2ros = False



    def mode2_control(self):

        self.rotate_end_effector()
        
        self.go_down_to_barrier()

        self.grip_on_barrier()        

        self.go_back_to_standby_pos()

        self.go_to_garbage_dump() 

        self.go_down_garbage_dump()

        self.grip_off_barrier()

        self.go_to_prev_pos()

        self.go_back_to_standby_pos()

        # self.go_down_for_ready()

        self.check_mode_change_state()
    

    def run(self):
        
        self.initialization()

        mouseImg = np.full((320, 240, 3), 255, dtype=np.uint8)
        
        cv2.namedWindow("Mouse click")
        cv2.imshow("Mouse click", mouseImg)
        cv2.setMouseCallback("Mouse click", self.mouse_event, mouseImg)
 
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안

            if self.MODE == self.MODE1:

                if self.flag_recv_msg_ms == True and self.flag_recv_msg_re == True:

                    self.check_mode()

                    self.mode1_control()
            
            elif self.MODE == self.MODE2:             
                
                if self.flag_recv_msg_re == True and self.flag_recv_msg_ard2ros == True:

                    self.mode2_control()
            
            cv2.imshow("Mouse click", mouseImg)
            cv2.waitKey(10)

            
            

           
def main():
    try:
        UR5e = UR5e_Move_With_Camera()
        UR5e.run()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()