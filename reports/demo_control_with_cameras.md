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



#### demo_control_with_cameras.py

It is a code that controls the robot arm using the values given by the camera. 

* **Code description**

  * Mode 1 robot control

    ```python
    def mode1_control(self):
        ...
        # Set minimum range to move robot
        if abs(self.cmd_x_ms - self.pre_cmd_x) > 0.005 or abs(self.cmd_y_ms - self.pre_cmd_y) > 0.005: 
           
            # Move robot end-effector with constant height
            target_pose_abs_xyz = [self.cmd_x_ms, self.cmd_y_ms, self.cur_pose[2]] 
    ```

    

  * Mode change flag

    ```python
    def mouse_click(self, event, x, y, flags, param):
            if event == cv2.EVENT_FLAG_L_BUTTON: # Mouse left button clicked and debris is detected -> Mode change 1 to 2
                self.MODE = self.MODE2
                print("MODE CHANGED..!")
    ```

    

  * End-effector rotation

    ```python
    def rotate_end_effector(self):
        # Rotate robot end-effector using detected debris angle value
        while(abs(self.cmd_angle_end_effector) > 0.1):
                    ...
                    if self.cmd_angle_end_effector < 0:
                        ...
                        cur_joint[5] = 1.13 * (cur_joint[5] + self.cmd_angle_end_effector)    
                    else:
                        cur_joint[5] = cur_joint[5] + self.cmd_angle_end_effector
    ```

    This is code for reading the angle of an object in Mode 2 to align it with the gripper. If the difference in angle is 0.1[rad], the code is set so that the gripper goes down and picks up the object. Different numbers are set depending on the angle (+,-) of the object. If the angle is not right in the new environment, you can use it by changing it to the appropriate value through experiments.

    

  * End-effector down

    ```python
    def go_down_to_barrier(self):
        	...
            target_pose[2] = cur_pose[2] - self.sensor_dist1 + 0.12 # 0.12 is obtained experimentally
    ```

    This is a code that allows Mode 2 to go down by the distance value of the obstacle. If this also doesn't work well in the new environment, you can change it to the appropriate value and use it. (Unit: [m])

    

  * Grip-off location

    ```python
    def go_down_garbage_dump(self):
            ...
            if self.i == 0:
                self.target_pose[0] = cur_pose[0] + 0.15
            elif self.i == 1:
                self.target_pose[0] = cur_pose[0] - 0.05
            elif self.i == 2:
                self.target_pose[0] = cur_pose[0] - 0.22
            ...
            # 물체를 내려 놓기 위한 Z 위치
            self.target_pose[2] = cur_pose[2] - 0.27
    ```

    This is a code for putting down obstacles. Currently, we are supposed to put the object down in the three positions we set. If necessary, you can change the x and z values and set the location where you want to drop them. (Unit: [m])

