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



#### Image_processing_for_roboteye.py

It is a code that checks the angle of an object through a camera attached to the robot arm.

* **Code description**

  * Debris angle calculation

  ```python
  def calculate_aspect_ratio(self, box): # angle can be calculated only by knowing where the long side is.
      # Function to calculate the aspect ratio of the given cuboid
      edge1 = np.linalg.norm(box[0] - box[1])
      edge2 = np.linalg.norm(box[1] - box[2])
      edge3 = np.linalg.norm(box[2] - box[3])
      edge4 = np.linalg.norm(box[3] - box[0])
  
      max_length = max(edge1, edge2, edge3, edge4)
      min_length = min(edge1, edge2, edge3, edge4)
  
      aspect_ratio = max_length / min_length
  ```

  ```python
   def calculate_angle(self, box):
          # Function to calculate the angle between the longest edge of the given cuboid
          edge_lengths = [np.linalg.norm(box[i] - box[(i + 1) % 4]) for i in range(4)]
          longest_edge_index = np.argmax(edge_lengths) #  Find the longest side
  
          # Find the coordinates of the two vertices of the longest side.
          vertex1 = box[longest_edge_index][0]
          vertex2 = box[(longest_edge_index + 1) % 4][0]
          
          angle = math.atan2(vertex2[1] - vertex1[1], vertex2[0] - vertex1[0]) * 180 / math.pi
          angle -= 90  # Subtract 90 degrees to make the y-axis the reference for 0 degree
          
          if angle < -90:
              angle += 180
          
          elif angle > 90:
              angle -= 180
  ```

