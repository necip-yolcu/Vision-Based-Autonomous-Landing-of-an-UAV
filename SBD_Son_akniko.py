from picamera.array import PiRGBArray
from picamera import PiCamera

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions

import time
#from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal

import cv2
import sys
import imutils
import math
import numpy as np

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]

    #-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
#-- Key event function
#def key(event):
#    if event.char == event.keysym: #-- standard keys
#        if event.keysym == 'r':
#            print("r pressed >> Set the vehicle to RTL")
#            vehicle.mode = VehicleMode("RTL")
#            
#    else: #-- non standard keys
#        if event.keysym == 'Up':
#            set_velocity_body(vehicle, gnd_speed, 0, 0)
#        elif event.keysym == 'Down':
#            set_velocity_body(vehicle,-gnd_speed, 0, 0)
#        elif event.keysym == 'Left':
#            set_velocity_body(vehicle, 0, -gnd_speed, 0)
#        elif event.keysym == 'Right':
#            set_velocity_body(vehicle, 0, gnd_speed, 0)
    
        
    #---- MAIN FUNCTION
    #- Takeoff
arm_and_takeoff(1)

camera = PiCamera()
#camera.resolution = (320, 240)
camera.resolution = (640, 480)
camera.framerate = 32
#rawCapture = PiRGBArray(camera, size=(320, 240))
rawCapture = PiRGBArray(camera, size=(640, 480))



for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame.array
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    #frame = cv2.cvtColor(np.asarray(frame), cv2.COLOR_BGR2GRAY)
    frame = cv2.dilate(frame, kernel, iterations=1)
    frame = cv2.erode(frame, kernel, iterations=2)
    frame = cv2.dilate(frame, kernel, iterations=1)  
    frame = cv2.convertScaleAbs(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    ######################COLOR######################
    yellowMin = (20,100,100)
    yellowMax = (30, 255, 255)

    blueMin=(50,100,100)
    blueMax=(100,255,255)

    brownMin=(0,100,0)
    brownMax=(20,255,255)
    
    yellow=cv2.inRange(hsv,yellowMin, yellowMax)
    blue=cv2.inRange(hsv,blueMin,blueMax)
    brown=cv2.inRange(hsv,brownMin,brownMax)  
    #####################################################
    
    #########################PARAMS###################################################
    params = cv2.SimpleBlobDetector_Params()  #<class 'cv2.SimpleBlobDetector_Params'> 
    params.minDistBetweenBlobs = 15.0  # minimum 10 pixels between blobs  
    params.filterByArea = True         # filter my blobs by area of blob
    params.minArea = 150 #r=9px           # min 1000 pixels squared
    params.maxArea = 10000  #r=40px   (print(.size)=80px=diameter         # max 10000 pixels squared
       
    params.minThreshold = 20
    params.maxThreshold = 200
    params.thresholdStep = 10    
    
    ##################################################################################    
    
    detector = cv2.SimpleBlobDetector_create(params) #<class 'cv2.SimpleBlobDetector'>
    keypoints = detector.detect(frame)   #<class 'list'>
    #keypoints=detector.detect(255-yellow)
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #params=cv2.SimpleBlobDetector_create()
#    cv2.line(im_with_keypoints, (160,0), (160,289), (255,0,0), 1)
#    cv2.line(im_with_keypoints, (0,120), (319,120), (255,0,0), 1)
    cv2.line(im_with_keypoints, (320,0), (320,640), (255,0,0), 1)
    cv2.line(im_with_keypoints, (0,240), (640,240), (255,0,0), 1)
           
    centre=[]
    diameter=[]
    t=[]
 
    for i in keypoints:
        #print("1")
        
        x=i.pt[0]; y=i.pt[1]
        d=i.size   #use this if(there are 3 blobs with size/distance=k)--->then do;
        centre.append([x,y])
        
     
            
        if len(centre) > 0:
            
            #print("A")
            
            first_center = centre[0]
            x1 = first_center[0]
            y1 = first_center[1]
            #print("1. center: ",first_center)
          
            diameter.append(d)
                       
        if len(centre) > 1:
            
            #print("B")
           
            second_center = centre[1]
            x2 = second_center[0]
            y2 = second_center[1]
            #print("2. center: ",second_center)
            t1 = math.sqrt(abs(x2 - x1) ** 2 + (abs(y2 - y1)) ** 2)
            #print("distance: ", t1)
           

        if len(centre) > 2:    
            #print("C")                
            third_center = centre[2]
  
            x3 = third_center[0]
            y3 = third_center[1]
            #print("3. center: ",third_center)
            
            Xc=(x1+x2+x3)/3
            Yc=(y1+y2+y3)/3
        
            print("XC",Xc)
            print("YC",Yc)
            
            
            ##im_with_keypoints=cv2.circle(im_with_keypoints, (int(Xc), int(Yc)),3, (255, 255, 0), 0)
            
            
           
            cv2.line(im_with_keypoints, (320,240), (int(Xc),int(Yc)), (0,255,0), 1)
            cv2.line(im_with_keypoints, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,0), 1)
            cv2.line(im_with_keypoints, (int(x1),int(y1)), (int(x3),int(y3)), (0,255,0), 1)
            cv2.line(im_with_keypoints, (int(x2),int(y2)), (int(x3),int(y3)), (0,255,0), 1)
        
        
        
        
        #duration=0.5
        while 0:
            #r=radius
            r=10
            if Xc>320+r:
                if Yc>240+r:
                    velocity_x=5
                    velocity_y=-5
                    velocity_z=0
                    set_velocity_body(vehicle, gnd_speed, -gnd_speed, 0)
                
                    print('4.bolge(DRONE guney-dogu yonune ilerliyor)')
                    
                elif Yc<240-r:
                    velocity_x=5
                    velocity_y=5
                    velocity_z=0
                    set_velocity_body(vehicle, gnd_speed, gnd_speed, 0)
                    
                    print('1.bolge( DRONE kuzey-dogu yonune ilerliyor)')
                    
                else:
                    velocity_x=5
                    velocity_y=0
                    velocity_z=0
                    set_velocity_body(vehicle, gnd_speed, 0, 0)
                    
                    print('DRONE sadece doguya ilerliyor')
                    
            elif Xc<320-r:
                if Yc>240+r:
                    velocity_x=-5
                    velocity_y=-5
                    velocity_z=0
                    set_velocity_body(vehicle, -gnd_speed, -gnd_speed, 0)
                    
                    print('3.bolge(DRONE guney-bati yonune ilerliyor)')
                     
                elif Yc<240-r:
                    velocity_x=-5
                    velocity_y=5
                    velocity_z=0
                    set_velocity_body(vehicle, -gnd_speed, gnd_speed, 0)
                
                    print('2.bolge(DRONE kuzey-bati yonune ilerliyor)')
                    
                else:
                    velocity_x=-5
                    velocity_y=0
                    velocity_z=0
                    set_velocity_body(vehicle, -gnd_speed, 0, 0)
                    
                    print('DRONE sadece batiya ilerliyor')
                    
            elif 320-r<Xc<320+r:
                if Yc>120+r:
                    velocity_x=0
                    velocity_y=-5
                    velocity_z=0
                    set_velocity_body(vehicle, 0, -gnd_speed, 0)
                    
                    print('DRONE sadece guneye ilerliyor')
                    
                elif Yc<240-r:
                    velocity_x=0
                    velocity_y=5
                    velocity_z=0
                    set_velocity_body(vehicle, 0, gnd_speed, 0)
                    
                    print('DRONE sadece kuzeye ilerliyor')
                    
                else:
                    velocity_x=0
                    velocity_y=0
                    velocity_z=0
                    set_velocity_body(vehicle, 0, 0, 0)
                    print('velocity_x',velocity_x)
                     
                    print('DRONE durup alcalmaya basliyor')
                 
    
    cv2.imshow('final', im_with_keypoints)

    key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
        

    


        
  
        

