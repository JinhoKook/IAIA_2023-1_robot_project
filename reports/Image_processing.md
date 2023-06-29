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



#### Image_processing.py

It is a code that obtain coordinates of an object of a specific color through a camera in a tracking stage.

* **Code description**

  * Color segmentation

  ```python
      def __init__(self):
  
  ...
  
          self.lower_bound = np.array([0, 192, 37])	# Initial color lower bound
          self.upper_bound = np.array([66, 255, 193]) # Initial color upper bound
  
          self.cap_ms = cv2.VideoCapture(4)
  ```

  ​	Lower bound and upper bound are the parts that define the initial HSV color boundary values, but if the specific color is not recognized properly, it can be adjusted through the track bar.

  

  * Image size & Track bar 

  ```python
  def ImaegProcessing(self):
  
      self.cap_ms.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
      self.cap_ms.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
  
      ...
  
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
  
          mask = cv2.inRange(hsv_image, self.lower_bound, self.upper_bound) # extracts a specific color boundary
          result = cv2.bitwise_and(image, image, mask=mask)
  
          contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
  ```

  ​	The size of the image was adjusted to 320 x 240 [pixels]. Too large image sizes causes large amount of computer processing, so we are set small image size. **Recommended not to change this size because it applies to pixel calculations of subsequent codes**

  

  * Coordinate transformation 

```python
            for contour in contours:

                if cv2.contourArea(contour) > 200: # Set minimum size of detected object
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3) # Draw rectangle as green.


                    self.flag_find = True

                    self.cx = ((x + int(w / 2)) - 160)/700 + 0.13 	# 0.13: Initial x point of robot end-effector
                    
                    self.cy = (240 - (y + int(h / 2)))/500 + 0.49 	# 0.49: Initial 4 point of robot end-effector
					
                   	 												# (cx, cy): pixel to robot absolute coordinate
                    
                    self.radi = math.sqrt(self.cx ** 2 + self.cy ** 2)  

                    if self.radi > 0.82:  # Set maximum range of robot move
                        self.cx = self.cx_prev
                        self.cy = self.cy_prev
                    
                    self.cx_prev = self.cx
                    self.cy_prev = self.cy
                else:
                    self.flag_find = False
                    self.cx = self.cx_prev
                    self.cy = self.cy_prev
			
            ...
            
            image = cv2.rotate(image, cv2.ROTATE_180) # Rotate image for user-friendly view
```


