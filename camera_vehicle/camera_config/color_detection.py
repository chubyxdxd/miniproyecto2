#color_detection.py
#
#

#
#

import cv2
import numpy as np
class color_detection_mask():
    def __init__(self, frame, color, desired_shape=None):
        self.frame = frame
        self.color = color
        self.desired_shape = desired_shape
        #self.command = command
        self.hsv_ranges = {
            "blue": (
                ((100, 100, 50), (140, 255, 255)),
            ),
            "red": (
                ((0, 100, 100), (10, 255, 255)),
                
            ),
            "green": (
                ((35, 100, 100), (85, 255, 255)),
            ),
            "yellow": (
                ((20, 100, 100), (30, 255, 255)),
            ),
            "pink": (
                ((140, 100, 100),(170, 255, 255)),
            )
        }
        self.name_tags = {
            2:"None",
            3:"triangle",
            4:"square",
            5:"pentagon",
            6:"hexagon",
            7:"circle"
        }
        self.shape = None
        self.cx = 0
        self.cy = 0
        self.band = 0

    def normal_thresh(self, th, max):
        gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        _, n_thresh = cv2.threshold(blurred_frame, th, max, cv2.THRESH_BINARY_INV)
        return n_thresh

    def adaptive_thresh(self, max):
        gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)       
        a_thresh = cv2.adaptiveThreshold(gray_frame, max, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 17, 19)
        return a_thresh
    def geometry_shape(self, vertices, w, h):
        if vertices in range(1,20):
            value = vertices
            if value >= 7:
                value = 7
            return self.name_tags[value], value
        else:
            return self.name_tags[2], 2
        
    def color_mask(self):
        #MASK FOR THE ENTIRE COLOR SELECTED
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        ranges = self.hsv_ranges.get(self.color, [])
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        filtered = cv2.bitwise_and(self.frame, self.frame, mask=combined_mask)
        
        # FOR PREPROCESSING contourns
        grayc = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(grayc, 10, 150)
        canny = cv2.dilate(canny, None, iterations = 1)
        canny = cv2.erode(canny, None, iterations = 1)
        cnts, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = filtered.copy()
        
        filtered_cnts = [c for c in cnts if cv2.contourArea(c) > 500]
        if not filtered_cnts:
            self.shape = "Nothing"
            self.cx = 0
            self.cy = 0
            #print(f"object {self.shape} : {self.cx}, {self.cy}")
            return output
        #ONLY WORKIN WITH THE BIGGEST shape
        max_contour = max(filtered_cnts, key=cv2.contourArea)

        epsilon = 0.02 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        cv2.drawContours(output, [approx], -1, (255,255,255), 8)
        vertices = len(approx)
        x,y,w,h = cv2.boundingRect(approx)
        self.shape, aux = self.geometry_shape(vertices, w, h)
        cv2.putText(output, str(vertices) + " " + self.shape, (x,y -10), cv2.FONT_HERSHEY_SIMPLEX,
                       1.2, (255,255,255), 4)
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            self.cx = int(M["m10"] / M["m00"])
            self.cy = h
        else:
            self.cx = 0
            self.cy = 0
        if aux is not None and self.desired_shape == aux:
            text = "SI BRO"
            self.band = 1
        else:
            text = "NO CORRESPONDE"
            self.band = 0
        print()
        cv2.circle(output, (self.cx,self.cy), 10, (0, 255, 0), -1)
        cv2.putText(output, f"({self.band},{self.cx},{self.cy})", (self.cx + 10, self.cy), cv2.FONT_HERSHEY_SIMPLEX,
                       1.2, (0, 255, 0), 4)
        
        #print(f"object {self.shape} : {self.cx}, {self.cy}")
        return output
        


