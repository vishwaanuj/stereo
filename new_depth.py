import io
import cv2
import webbrowser
from PIL import Image
import numpy as np
from base64 import b64encode
import base64
import eel
import os
from imutils import resize
from datetime import date
eel.init('web')
from imutils.video import WebcamVideoStream,VideoStream

global save
save=0#for saving of frame after stopping the video stream
streaming = False#will be the flag for showing frame or not,basically it will act as toggle 
cap=VideoStream(src=0,resolution=(320, 240),
		framerate=32).start()#resolution is not working for the camera

global frame
frame=0#normal global frame,is not required tbh


#-------for eel -----
host = "localhost"
port = 8000
html = "index.html"
url = "http://{}:{}/{}".format(host, port, html)
#------------------


border=1

window_size = 15  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

left_matcher = cv2.StereoSGBM_create(
        minDisparity=32,
        numDisparities=144,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=5,
        P1=16 * 3 * window_size ** 2,
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
lmbda = 70000
sigma = 1.2
visual_multiplier = 1.0

wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

print("starting the frame")
#cap = cv2.VideoCapture(0)
@eel.expose
def toggle_stream():
    global streaming
    streaming = not streaming
    if streaming == True:
        print("Streaming start.")
    else:
        print("Streaming stopped.")
def depth_calc(frame):
    frame=cv2.resize(frame, (750,400))
    height,width,_ = frame.shape
    
    half_width = int(width / 2)  
    left = frame[0:height,0:half_width]
    right = frame[0:height,half_width:width]
    

    imgL = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg,
                                beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    equ = np.uint8(filteredImg)
    #disp = cv2.normalize(stereo.compute(left,right),disparity, alpha=0, beta=255,norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    equ = cv2.equalizeHist(equ)
    height,width = equ.shape
    equ=equ[0:height,int(width/2):width]
    #equ=equ[0:height,width/2:width]
    
    return equ
from datetime import datetime

# datetime object containing current date and time
import os
global depth_save
@eel.expose
def save_image():
    today=datetime.now()
    d4 = today.strftime("%b-%d-%Y")
    current_time=today.strftime("%H:%M:%S")

    if not os.path.exists('images/'+d4):
        os.makedirs('images/'+d4)
    print("d =", d4)
    cv2.imwrite('images/'+d4+'/'+current_time+'.png',save)
    cv2.imwrite('images/'+d4+'/'+current_time+'_depth.png',depth_save)
    print('frame saved')
    eel.alert_save()
    #eel.alert_save()

def loop():
    global save
    global depth_save
    while True:
        frame = cap.read()
        depth=depth_calc(frame)
        frame=cv2.resize(frame,(400,400))
        depth=cv2.resize(depth,(400,400))
        if streaming == True:
            save=frame
            depth_save=depth
            imshow(frame)
            imshow_d(depth)
        eel.sleep(0.25)
        
def imshow_d(frame):
    ret, jpeg = cv2.imencode('.jpg', frame)
    jpeg_b64 = base64.b64encode(jpeg.tobytes())
    jpeg_str = jpeg_b64.decode()
    eel.js_imshow_d(jpeg_str)
def imshow(frame):
    ret, jpeg = cv2.imencode('.jpg', frame)
    jpeg_b64 = base64.b64encode(jpeg.tobytes())
    jpeg_str = jpeg_b64.decode()
    eel.js_imshow(jpeg_str)
eel.spawn(loop)
eel.start(html, port=port, host=host)
#webbrowser.get('firefox').open_new_tab(url)