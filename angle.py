#import libs
import cv2
import numpy as np
import math

def distance((x1, y1), (x2,y2)):
    dist = math.sqrt((math.fabs(x2-x1))**2+((math.fabs(y2-y1)))**2)
    return dist
 
#filters for blue color and returns blue color position.
def findblue(frame):
    maxcontour = None
    blue = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bluelow = np.array([55, 74, 0])#replace with your HSV Values
    bluehi = np.array([74, 227, 255])#replace with your HSV Values
    mask = cv2.inRange(blue, bluelow, bluehi)
    res = cv2.bitwise_and(frame, frame, mask=mask)
 
#filters for orange color and returns orange color position.
def findorange(frame):
    maxcontour = None
    orange = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orangelow =  np.array([0, 142, 107])#replace with your HSV Values
    orangehi = np.array([39, 255, 255])#replace with your HSV Values
    mask = cv2.inRange(orange, orangelow, orangehi)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
def findblue(frame):
    blue = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bluelow = np.array([55, 74, 0])#replace with your HSV Values
    bluehi = np.array([74, 227, 255])#replace with your HSV Values
    mask = cv2.inRange(blue, bluelow, bluehi)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cnts, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts) &amp;amp;amp;gt;0:
        maxcontour = max(cnts, key = cv2.contourArea)
 
        #All this stuff about moments and M['m10'] etc.. are just to return center coordinates
        M = cv2.moments(maxcontour)
        if M['m00'] &amp;amp;amp;gt; 0 and cv2.contourArea(maxcontour)&amp;amp;amp;gt;2000:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy), True
        else:
            #(700,700), arbitrary random values that will conveniently not be displayed on screen
            return (700,700), False
    else:
        return (700,700), False
    
#filters for orange color and returns orange color position.
def findorange(frame):
    orange = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orangelow =  np.array([0, 142, 107])#replace with your HSV Values
    orangehi = np.array([39, 255, 255])#replace with your HSV Values
    mask = cv2.inRange(orange, orangelow, orangehi)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cnts, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts) &amp;amp;amp;gt;0:
        maxcontour = max(cnts, key = cv2.contourArea)
        M = cv2.moments(maxcontour)
        if M['m00'] &amp;amp;amp;gt; 0 and cv2.contourArea(maxcontour)&amp;amp;amp;gt;2000:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy), True
        else:
            return (700,700), False
    else:
        return (700,700), False
    
#uses distance formula to calculate distance

#capture video
cap = cv2.VideoCapture(0)
 
while(1):
    _, frame = cap.read()
#if you're sending the whole frame as a parameter,easier to debug if you send a copy
    fra = frame.copy() 
 
     #get coordinates of each object
    (bluex, bluey), blogic = findblue(fra)
    (orangex, orangey), ologic = findorange(fra)
    #draw two circles around the objects (you can change the numbers as you like)
    cv2.circle(frame, (bluex, bluey), 20, (255, 0, 0), -1)
    cv2.circle(frame, (orangex, orangey), 20, (0, 128, 255), -1)
    
if blogic and ologic:
    #quantifies the hypotenuse of the triangle
    hypotenuse =  distance((bluex,bluey), (orangex, orangey))
    #quantifies the horizontal of the triangle
    horizontal = distance((bluex, bluey), (orangex, bluey))
    #makes the third-line of the triangle
    thirdline = distance((orangex, orangey), (orangex, bluey))
    #calculates the angle using trigonometry
    angle = np.arcsin((thirdline/hypotenuse))* 180/math.pi
 
    #draws all 3 lines
    cv2.line(frame, (bluex, bluey), (orangex, orangey), (0, 0, 255), 2)
    cv2.line(frame, (bluex, bluey), (orangex, bluey), (0, 0, 255), 2)
    cv2.line(frame, (orangex,orangey), (orangex, bluey), (0,0,255), 2)
    
     #Allows for calculation until 180 degrees instead of 90
if orangey &amp;amp;lt; bluey and orangex &amp;amp;gt; bluex:
    cv2.putText(frame, str(int(angle)), (bluex-30, bluey), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1, (0,128,220), 2)
elif orangey &amp;amp;lt; bluey and orangex &amp;amp;lt; bluex:
    cv2.putText(frame, str(int(180 - angle)),(bluex-30, bluey), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1, (0,128,220), 2)
elif orangey &amp;amp;gt; bluey and orangex &amp;amp;lt; bluex:
    cv2.putText(frame, str(int(180 + angle)),(bluex-30, bluey), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1, (0,128,220), 2)
elif orangey &amp;amp;gt; bluey and orangex &amp;amp;gt; bluex:
    cv2.putText(frame, str(int(360 - angle)),(bluex-30, bluey), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1, (0,128, 229), 2)
if k == ord('q'): break