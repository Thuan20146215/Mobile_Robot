import cv2
import numpy as np
 
frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
 

from picamera2 import Picamera2
def init_Cam():
    global picam2
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,360)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()

    picam2.configure("preview")
    picam2.start()
    return picam2

def empty(a):
    pass
 
cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)
cv2.createTrackbar("HUE Min", "HSV", 0, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 179, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 0, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 0, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)
 
def init_camera():
  global cap
  try:
      cap = cv2.VideoCapture(0)
      url = "http://192.168.100.38:8080/video"  # IP address of your phone
      cap.open(url)

      return cap
  except Exception as e:
      print(f"Error initializing camera: {e}")
      return None
frameCounter = 0
init_Cam()
#cap = cv2.VideoCapture(0)
while True:
    # frameCounter +=1
    # if cap.get(cv2.CAP_PROP_FRAME_COUNT) ==frameCounter:
    #     cap.set(cv2.CAP_PROP_POS_FRAMES,0)
    #     frameCounter=0
 
    # _, img = cap.read()
    #success, img = cap.read()
    img= picam2.capture_array()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
 
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
    print(h_min)
 
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
 
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    hStack = np.hstack([img, mask, result])
    cv2.imshow('Horizontal Stacking', hStack)
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
 
# cap.release()
# cv2.destroyAllWindows()