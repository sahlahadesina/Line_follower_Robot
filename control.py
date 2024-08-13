import cv2
import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._previous_error = 0
        self._integral = 0

    def compute(self, measurement):
        error = self.setpoint - measurement
        self._integral += error
        derivative = error - self._previous_error
        self._previous_error = error
        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative

pid_angle = PID(Kp=0.3, Ki=0.0, Kd=0.001)  # PID controller for angle
pid_position = PID(Kp=0.3, Ki=0.0, Kd=0.001)  # PID controller for position

def process_frame(frame):
    t = 0
    # Convert the image to grayscale
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ground = cv2.inRange(hsv, (0, 0, 0), (90,255,255))
    _, binaryGround = cv2.threshold(ground, 40, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(binaryGround, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        x,y,w,h  = cv2.boundingRect(max_contour)
    # lines = cv2.HoughLinesP(binaryGround,1,np.pi/180,40,minLineLength=frame.shape[0]/5,maxLineGap=15)
        
        

        contours, _ = cv2.findContours(binaryGround[h:,:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
                max_contour = max(contours, key=cv2.contourArea)
                x1,y1,w1,h1  = cv2.boundingRect(max_contour)
    # x,y,w,h = cv2.boundingRect(contours)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if h > 0 and y1>0:
                    gray = gray[h:h+y1,:]
                    t=h
                    cv2.rectangle(frame,(x,y+h),(x+w,y+h+y1),(0,255,0),2)

        

    # gray = gray[0:y, 0:y+h]
    
    # Apply GaussianBlur to reduce noise
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply thresholding to get a binary image
    _, binary = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour (presumably the line)
        max_contour = max(contours, key=cv2.contourArea)
        rows, cols = gray.shape[:2]
        
        # Fit a line using the least squares method
        [vx, vy, x, y] = cv2.fitLine(max_contour, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y+t)
        righty = int(((cols - x) * vy / vx) + y+t)
        
        try:
            cv2.line(frame, (cols - 1, righty), (0, lefty), (255, 0, 0), 2)
        except:
            return None, None
        
        # Calculate the angle of the line
        angle = np.arctan2(vy, vx)
        
        # Find the center of the rectangle
        rect = cv2.minAreaRect(max_contour)
        center = (int(rect[0][0]), int(rect[0][1]+t))
        
        # Draw the center on the image
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        
        # Display the frame
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)  # Short delay for proper display

        return angle, center
    else:
        # Display the frame
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)  # Short delay for proper display
        
        return None, None

def control_robot(d, angle, center, frame_center, pid_angle, pid_position, max_speed):
    if angle is not None and center is not None:
        # Calculate the correction for angle and position
        angle_correction = pid_angle.compute(angle)
        position_correction = pid_position.compute(center[0] - frame_center)
        if abs(angle)>0.3:
            speed =  1 
        else:
            speed = 1  

        # Combine the corrections
        correction = angle_correction + position_correction
        
        base_speed = max_speed / 2
        left_wheel_speed = base_speed + correction
        right_wheel_speed = base_speed - correction
        
        # Limit speed and smoothness
        left_wheel_speed = np.clip(left_wheel_speed, -max_speed, max_speed)
        right_wheel_speed = np.clip(right_wheel_speed, -max_speed, max_speed)
        
        d.ctrl[0] = left_wheel_speed * speed
        d.ctrl[1] = right_wheel_speed * speed

def control(renderer, d):
    renderer.update_scene(d, "cam")
    img = np.array(renderer.render())
    angle, center = process_frame(img)
    if angle is not None and center is not None:
            control_robot(d, angle, center, 300 , pid_angle, pid_position , 40 )
