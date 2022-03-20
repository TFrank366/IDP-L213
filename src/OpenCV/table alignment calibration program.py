from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import ctypes
from imutils.perspective import four_point_transform

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
## get Screen Size
user32 = ctypes.windll.user32
screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

W,H = screensize
    
#stream = cv.VideoCapture('http://localhost:8081/stream/video.mjpeg')

DIM=(1016, 760)
K=np.array([[585.2033685583266, 0.0, 521.6451976959074], [0.0, 583.348171416934, 382.5660809831434], [0.0, 0.0, 1.0]])
D=np.array([[-0.13339260676514814], [0.3857155669436128], [-0.5989247214760174], [0.3289234211743555]])

def undistort(img, balance=0.3, dim2=None, dim3=None):
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv.CV_16SC2)
    undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    return undistorted_img


while True:
    #ret, frame = cap.read()
   # frame = cv.imread('c:/Users/Harry/OneDrive - University of Cambridge/Documents/Cambridge/Course related/Second Year/IDP/Software/example_idpcam1.jpg')
    frame = cv.imread('c:/Users/Harry/OneDrive - University of Cambridge/Documents/Cambridge/Course related/Second Year/IDP/Software/q5.jpg')
    #ret,frame = stream.read()

    if frame is None:
        break
    undistorted = undistort(frame)


    height, width = undistorted.shape[:2]
    Blank = np.zeros((height,width,3), np.uint8)
    Blank2 = np.zeros((height,width,3), np.uint8)
    gray = cv.cvtColor(undistorted, cv.COLOR_BGR2GRAY) # convert the image to gray scale
    image_enhanced = cv.equalizeHist(gray)

    clahe = cv.createCLAHE(clipLimit = 5)
    final_img = clahe.apply(image_enhanced) + 30

    ret,thresh1 = cv.threshold(image_enhanced,127,255,cv.THRESH_BINARY)
    ret,thresh2 = cv.threshold(image_enhanced,127,255,cv.THRESH_BINARY_INV)
    ret,thresh3 = cv.threshold(image_enhanced,127,255,cv.THRESH_TRUNC)
    ret,thresh4 = cv.threshold(image_enhanced,127,255,cv.THRESH_TOZERO)
    ret,thresh45 = cv.threshold(thresh4,150,255,cv.THRESH_BINARY)
    ret,thresh5 = cv.threshold(image_enhanced,127,255,cv.THRESH_TOZERO_INV)

    Hori = np.concatenate((thresh1, final_img,thresh3), axis=1)
    Hori2 = np.concatenate((thresh4, thresh45,thresh5), axis=1)

    # concatanate image Vertically
    Verti = np.concatenate((Hori, Hori2), axis=0)
    height, width = Verti.shape

    scaleWidth = float(W)/float(width)
    scaleHeight = float(H)/float(height)
    if scaleHeight>scaleWidth:
          imgScale = scaleWidth
    else:
          imgScale = scaleHeight

    newX,newY = Verti.shape[1]*imgScale, Verti.shape[0]*imgScale
    newimg = cv.resize(Verti,(int(newX),int(newY)))
    cv.imshow('1', newimg)
    blur = cv.GaussianBlur(thresh3, (3, 3), 0)
    edged = cv.Canny(blur, 25, 250) # Apply the Canny algorithm to find the edges

    contours, _ = cv.findContours( edged, cv.RETR_TREE, cv.CHAIN_APPROX_NONE )
  
    i = 0
    
    # list for storing names of shapes
    mask = np.zeros_like(blur)
    for contour in contours:
       # area = cv.contourArea(contour)
       # if area < 100:
        #    cv.fillPoly(mask, [contour], 255)
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        #if i == 0:
         # print (contour)
          #  continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv.approxPolyDP(
            contour, 5, True)
        x= 500
        y=500
        # finding center point of shape
        M = cv.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        color = (0,255,0)

        if x<760 and y<1016:
            r,g,b = undistorted[x, y]
            color = (int(r),int(g),int(b))    



        # putting shape name at center of each shape
       #cv.putText(Blank, '.', (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (000, 255, 000), 2)
       # cv.putText(Blank, 'X', (821, 141), cv.FONT_HERSHEY_SIMPLEX, 0.5, (000, 255, 255), 2)
      #  cv.drawContours(Blank, [contour], 0, color, 2)
    

    #Find longest conotur (edge of table)
    perimeter=[]
    for cnt in contours[1:]:
        perimeter.append(cv.arcLength(cnt,True))
    maxindex= perimeter.index(max(perimeter))

    # Draw edge of table on blank image
    #cv.drawContours( Blank, contours, maxindex+1, (255,255,255), -1)
    approx = cv.approxPolyDP(contours[maxindex+1], 0.01*cv.arcLength(contours[maxindex +1], True), True)
    cv.drawContours( Blank, [approx], 0, (255,255,255), -1)
    #convert image edge to grey scale
    Binarised = cv.cvtColor(Blank, cv.COLOR_BGR2GRAY)
    cv.imshow("Binarised", Binarised)

    # find contours in edge image
    contours2, _ = cv.findContours( Binarised, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE )
    #print (contours2)
    
####################################################################################
###turn table into white rectangle##########
    for contour in contours2:
        hull = cv.convexHull(contour)
        cv.drawContours( Blank2, [hull], 0, (255,255,255), -1)

    grey_blank2 = cv.cvtColor(Blank2, cv.COLOR_BGR2GRAY)
    blank2_edge = cv.Canny(grey_blank2,75,200)
    cv.imshow("Blank2", blank2_edge)

####################################################################################
    contours3, _ = cv.findContours( blank2_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE )

    for cnt in contours3[1:]:
        perimeter.append(cv.arcLength(cnt,True))
    maxindex2= perimeter.index(max(perimeter))

    approx2 = cv.approxPolyDP(contours3[0], 0.01*cv.arcLength(contours2[0], True), True)
    print (approx2)

    warped = four_point_transform(undistorted, approx2.reshape(4, 2))
    # convert the warped image to grayscale
    cv.imshow("Scanned", warped)







    Hori3 = np.concatenate((undistorted, Blank), axis=1)

    cv.imshow(window_capture_name, Hori3)
    
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break