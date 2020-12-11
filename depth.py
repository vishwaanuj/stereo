import io
import cv2
import numpy as np
from base64 import b64encode
import base64
import eel
import os
eel.init('web')
from imutils.video import WebcamVideoStream
stream= WebcamVideoStream(src=0).start()
print("starting the frame")



 

border=1

window_size = 3  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=160,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=5,
        P1=8 * 3 * window_size ** 2,
        # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=0,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

# FILTER Parameters
lmbda = 80000
sigma = 1.2
visual_multiplier = 1.0

wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)


@eel.expose
def dummy(dummy_param):
    print("I got a parameter: ", dummy_param)
    return "string_value", 1, 1.2, True, [1, 2, 3, 4], {"name": "eel"}
def depth_calc(frame):
    height,width,_ = frame.shape

    half_width = int(width / 2)  
    left = frame[0:height,0:half_width-border]
    right = frame[0:height,half_width+border:width]

    imgL = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    filteredImg = np.uint8(filteredImg)
    #disp = cv2.normalize(stereo.compute(left,right),disparity, alpha=0, beta=255,norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    equ = cv2.equalizeHist(filteredImg)
    height,width = equ.shape
    equ=equ[0:int(height),int(width/4)+20:width]
    return equ

@eel.expose
def generate_qr(data):
    print(data)
    frame = stream.read()
    height,width,_ = frame.shape
    

    half_width = int(width / 2)  
    print("")
    copy=frame.copy()
    frame=cv2.resize(frame,(400, 400))
    copy= cv2.resize(copy,(1000, 420))#copy for disparity cuz original image width making it ugl
    #img = cv2.imread('disparity.png')
    img=depth_calc(copy)
   
    retval, buffer = cv2.imencode('.png', frame) #real frame
    retval, buffer1 = cv2.imencode('.png', img)#disparity image
    
    real = base64.b64encode(buffer).decode("ascii")
    depth = base64.b64encode(buffer1).decode("ascii")
   
    print("QR code generation successful.")
    image1="data:image/png;base64, " + real
    image2="data:image/png;base64, " + depth
    return (image1,image2)


eel.start('index.html', size=(2000, 2000))