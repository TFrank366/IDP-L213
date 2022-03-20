############    Import librarys 


from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from imutils.perspective import four_point_transform
from scipy.spatial.transform import Rotation as R
import serial
import time
import math
 
############    Set up global variables for fuducial marker tracking
aruco_marker_side_length = 0.01
this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
this_aruco_parameters = cv2.aruco.DetectorParameters_create()
  
############    Set up connection to camera
stream = cv2.VideoCapture('http://localhost:8081/stream/video.mjpeg')

############    Set up global variables for positions for targets on undistorted and aligned image
target0_coordinates = (593,340)#(541,280) # commented values are tue centres for targets, values being used have been corrected for parralax
target1_coordinates = (669,274) #(619,207)
target2_coordinates = (399,142)#(456,198)# (396,163)
target3_coordinates = (469,90)#(530,126) #(655,273)
target4_coordinates = (580,165)#(525,218)
crosshair1_cooridinates = (656,99)#(561,185)
crosshair2_cooridinates = (464,280) # (380,357) #(489,252)
home_coordinates= (688,59)
tablepixels = 692-25  
tablecm = 2.39
center_x = 0
center_y = 0
yaw_z = 0

############    Set up global variables for fisheye correction - values found using calibration program
DIM = (1016, 760)
K = np.array([[585.2033685583266, 0.0, 521.6451976959074], [0.0, 583.348171416934, 382.5660809831434], [0.0, 0.0, 1.0]])
D = np.array([[-0.13339260676514814], [0.3857155669436128], [-0.5989247214760174], [0.3289234211743555]])

############    Set up global variables for aruco camera correction - values found using aruco calibration program
mtx = np.array([[950.0087894  ,  0.  ,       333.20576598],
 [  0.    ,     886.96472856, 340.09506847],
 [  0.    ,       0.        ,   1.        ]])

dst = np.array([[ 2.40302725e+00, -1.17169760e+01, -7.51639181e-02, -8.23081161e-04,
   3.55165075e+01]])


############    Set up variables for serial connection to arduino

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False


############    Functions for serial communication - based of arduino example code for python communication modified to allow for bluetooth

def setupSerial(baudRate, serialPortName):
    '''This function initialises the serial communication'''
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))



def sendToArduino(stringToSend):
    '''this adds the start and end-markers and sends the string to arduino via bluetooth'''
    global startMarker, endMarker, serialPort
    #adds markers
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


def recvLikeArduino():
    '''This recieves the data from the arduino'''
    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" # used to indicate a blank or corrupted recieve

############    Functions for removing fisheye effect based off - https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
############    modified to allow for increased framerate, uses remaping as opposed to opencv.undistort and saves map as this is function of camera so does not need to be recalculated at each step

def undistort_origional(img, balance=0.3, dim2=None, dim3=None):
    ''' this function is run initially to remapp the first frame and produce the maps required later'''
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_img, map1,map2

def undistort(img,map1,map2):
    ''' use the maps generated in the origional function to remap the frame'''
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

########### This crops the image to use on the area in the table and also realigns it to be square, uses information from table alignment calibation program
########### as table does not usually move by a noticible amount between runs having saved values prevents issues with misidentifying edge of the table
def realign (img, corners = np.array([[194 , 16], [119, 716], [868 ,749], [859 , 42]],dtype = np.int32)):
    ''' aligns image to table'''
    warped = four_point_transform(img, corners.reshape(4, 2))
    return warped

############ 

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians


############    this is used to return the co-ordinates of the redblock on the undistorted image

def find_red_block (img):
    '''This is used to return the co-ordinates of the redblock on the undistorted image'''
    #setting up local variables
    # setting threshold values for colour masking
    low_H = 165
    low_S = 75
    low_L = 75
    high_H = 180
    high_S = 230
    high_L = 210
    # defaulting the position to 0 
    x =0
    y=0


    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)     # converting the image from rbg to HLS as this allows for better isolation of the block on the image mask
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_L, low_S), (high_H, high_L, high_S))
    cv2.imshow("red", frame_threshold) # displaying thresholded image to help with debugging

    blur = cv2.GaussianBlur(frame_threshold, (3, 3), 0)  #blur hleps to remove single pixel noise and improves contour detection
    contours, _ = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    numberofblocks = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 75 and area<300: # check that area is around that of the block to help eliminate false positives
            numberofblocks = numberofblocks+1

            # finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            if cx<180 and cy <690 and cy >500: # Check that the block is roughly in the area of the board - igroe red blocks theat are not in pick up zone
                y = cy
                x= cx

    cv2.imshow("Blur", blur)  # displaying blurred image to help with debugging
    if x != 0 or y !=0:
         return x,y
    else:
        return 0,0 #if block not detected return 0,0 as default




############ This is used to return the co-ordinates of the robot on the undistorted image 
############ origionally inspired by https://automaticaddison.com/how-to-detect-aruco-markers-using-opencv-and-python/ 
############ and https://automaticaddison.com/how-to-perform-pose-estimation-using-an-aruco-marker/
############ modified to combine together and remove superferlous components as well as simplify

def trackrobot(frame):
    '''returns robot co-ordinates and angle on frame given'''
    # use global variables to allow for comparisons between iterations
    global center_x
    global center_y
    global yaw_z
    # detect marker on robot
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
      frame, this_aruco_dictionary, parameters=this_aruco_parameters,cameraMatrix=mtx, distCoeff=dst)
       
    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        mtx,
        dst)
         
    #calculate pose for marker
      for i, marker_id in enumerate(ids):
       
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        transform_rotation_x = quat[0] 
        transform_rotation_y = quat[1] 
        transform_rotation_z = quat[2] 
        transform_rotation_w = quat[3] 
         
        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                       transform_rotation_y, 
                                                       transform_rotation_z, 
                                                       transform_rotation_w)
         
        roll_x = math.degrees(roll_x)
        pitch_y = math.degrees(pitch_y)
        yaw_z = math.degrees(yaw_z)

      # Flatten the ArUco IDs list
      ids = ids.flatten()
       
      # Loop over the detected ArUco corners
      for (marker_corner, marker_id) in zip(corners, ids):
       
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
         
        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
         
        # Draw the bounding box of the ArUco detection
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
         
        # Calculate and draw the center of the ArUco marker
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
        
    cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
    # Display the resulting frame
    cv2.imshow('robot',frame)
    return center_x,center_y,yaw_z


############ 
def angle(x1,y1,x2,y2):
    ''' find angle to block as a signed angle, 0 is vertical'''
    delta_x = x2-x1
    delta_y = y1-y2
    if delta_y != 0:
        ang = math.atan2(delta_x,delta_y)
        ang = math.degrees(ang)
        return ang
    else:
        return 0

############

def distance_to_target (x1,y1,x2,y2):
    '''calculates distance between two points'''
    distance = math.sqrt((x1-x2)**2+(y1-y2)**2)
    distance = distance *(tablecm/tablepixels) # convert distance to m 
    #print(distance)
    return distance


############

def main ():
    """
    Main method of the program.
    """       
    setupSerial(115200, "COM7")
    prevTime = time.time()
    target = 0
    i=0
    lastblock_x = 0 
    lastblock_y = 0
    start = input("start")  # using an input to halt the program until a arbitary value is entered
    #allows us to verify that the connections to bluetooth and the camera are working before the tinmer starts during the competition
    sendToArduino("go") # tells the robot to start 

    #the main loop for the program
    while True:
        arduinoReply = recvLikeArduino()

        ret,frame = stream.read() # recieves frame from stream
        if np.shape(frame) != (): # checks that the frame has been recieved

            if i == 0: # if first iteration then use full undistort code
                frame,map1,map2 = undistort_origional(frame)
                i = 1

            else: # otherwise use faster version
                frame = undistort(frame,map1,map2)
                
            frame = realign(frame) # cropps the image to the table
            kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            sharpened = cv2.filter2D(frame, -1, kernel) # sharpens the image to help with thte detection of the aruco tag
            if not (arduinoReply == 'XXX'):
                print ("recieved at: "+ str(time.time()) +str(arduinoReply))
                target = int(arduinoReply) # sets target to value recieved
                print("current target =  "+ str(target))

            # sets the target location depending on what the robot asked for
            if target == 0:
                target_x,target_y =  target0_coordinates[0],target0_coordinates[1]
            elif target == 1:
                target_x,target_y =  target1_coordinates[0],target1_coordinates[1]
            elif target == 2:
                target_x,target_y = target2_coordinates[0],target2_coordinates[1]
            elif target == 3:
                target_x,target_y = target3_coordinates[0],target3_coordinates[1]
            elif target == 4:
                target_x,target_y = target4_coordinates[0],target4_coordinates[1]
            elif target == 5:
                target_x,target_y =crosshair1_cooridinates[0],crosshair1_cooridinates[1]
            elif target == 6:
                target_x,target_y =crosshair2_cooridinates[0],crosshair2_cooridinates[1]
            else:
                block_x,block_y = find_red_block(frame) # finds the red block
                if block_x == 0 and block_y == 0: # if block undetected uses last known position
                    block_x = lastblock_x
                    block_y = lastblock_y
                lastblock_x = block_x
                lastblock_y = block_y
                target_x = block_x
                target_y = block_y


            robot_x,robot_y,robot_yaw = trackrobot(sharpened) # find the robot position and angle

            # fail safe in that if the robot asks for one of the drop off zones while being on the pick up side of the ramp it will attempt to first get to the centre of the area to prevent it beingstuck on the yellow barriers
            if (robot_x-200 < robot_y): 
                if target < 6:
                    target = 4

            cv2.line(frame,(robot_x,robot_y),(target_x,target_y) , (0, 255, 0), 2) # draws a line between the robot and the current target on the targeting screen
            robot_yaw = -robot_yaw # reverses robot yaw to be consistent with sign convention used on arduino
            if (robot_x!= 0 and robot_y != 0 and target_x !=0 and target_y != 0): # checks that target and robot have been detected 
                angletoblock = -angle (robot_x,robot_y,target_x,target_y) # reverses angle to target to be consistent with sign convention used on arduino
            else:
                angletoblock = robot_yaw # if robot or target haven't been detected then sets the angle to block to be same as the robot yaw to cause angle error to be equal to zero 

            if abs(robot_yaw)>170: # prevents issue where near 180 the sign of the angle fluctuates, causign the robot to be unsure as to direction to go in - value is potentailly wrong but as updated multiple times subsequent iterations eliminate this error
                robot_yaw = abs(robot_yaw)

            heading_error = angletoblock - robot_yaw # finds angle error

            dist_to_target = -distance_to_target(robot_x,robot_y,target_x,target_y) # find distance to target

            # find the smallest angle needed to turn through
            if heading_error > 180:
                heading_error = heading_error - 360
            if heading_error < -180:
                heading_error = heading_error + 360

            #reverse to targets after dropping blocks off 
            if target == 4 or target == 5 or target == 6:
                dist_to_target = - dist_to_target
                if heading_error > 0:
                    heading_error = heading_error-180
                else:
                    heading_error = heading_error+180

           # print ( "robot position" +str(robot_x)+" , "+str(robot_y)+" robot angle: "+str(robot_yaw)+" target location: " + str(target_x)+",  "+str(target_y) +" angle to target: "+ str (angletoblock)+ " heading error" + str (heading_error))

            command = (str(heading_error)+","+ str((dist_to_target))) 
            if not (arduinoReply == 'XXX'): # send information to arduino if asked for
                if time.time() > prevTime +0.05:
                    prevTime = time.time()
                    print (command)               
                    sendToArduino(command)
                    print("sent at"+ str(time.time()))
                    cv2.imshow("target",frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): # end program if q pressed
            break


if __name__ == '__main__': # starts the program
  print(__doc__)
  main()