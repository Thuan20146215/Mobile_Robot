import cv2
import numpy as np

###
def findContours(img, imgOriginal):
    x=0
    sub_contour_width=0
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        
        area = cv2.contourArea(cnt)
        if area > 1000:  # Adjust the threshold based on your requirements
         
            cv2.drawContours(imgOriginal, [cnt], -1, (255, 0, 255), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            sub_contour_width = w // 3
 

    # Create three smaller contours with the same height and different widths
            for i in range(3):
                sub_x = x + i * sub_contour_width
                sub_w = sub_contour_width
        
        # Draw rectangles for each sub-contour
                cv2.rectangle(imgOriginal, (sub_x, y), (sub_x + sub_w, y + h), (0, 255, 0), 2)
            cv2.rectangle(imgOriginal, (x, y), (x + w, y + h), (0, 0, 255), 2)
    return imgOriginal,x,sub_contour_width

def find_large_contours(image,imgOriginal, min_area=2000):
    # Convert the image to grayscale
    # Find contours
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # List to store the bounding boxes
    bounding_boxes = []
    
    for cnt in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(cnt)
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        if (area > min_area) and (w<240):
            # Compute the bounding box
            cv2.drawContours(imgOriginal, [cnt], -1, (255, 0, 255), 3)
            # peri = cv2.arcLength(cnt, True)
            # approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            # x, y, w, h = cv2.boundingRect(approx)
            bounding_boxes.append((x, y, w, h))
            cv2.rectangle(imgOriginal, (x, y), (x + w, y + h), (0, 0, 255), 2)
    return bounding_boxes,imgOriginal


def findContours1(img, imgOriginal,lane):
    x=y=w=h=i=0
    sub_contour_width=0
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]
            
    for cnt in contours:
        i+=1
        cv2.drawContours(imgOriginal, [cnt], -1, (255, 0, 255), 3)
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(imgOriginal, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if i == lane:
            return imgOriginal,x,y,w,h
    return imgOriginal,x,y,w,h,len(contours)
def smallcontour(imgO,x,y,w,h):
    cv2.rectangle(imgO, (x,y),(x+w,y+h),(255,0,0),2)
    return imgO
###
def thresholding(img):
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # lowerBlk = np.array([0,0,0])
    # upperBlk = np.array([179,255,96]) #good
    lowerBlk = np.array([0,0,0])
    upperBlk = np.array([179,255,97]) #good
    maskBlk = cv2.inRange(imgHsv,lowerBlk,upperBlk)
    return maskBlk
 
def warpImg(img,points,w,h,inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp
 
def nothing(a):
    pass
 

############# WIP   
def initializeTrackbars1(intialTracbarVals,wT=480, hT=240):
    cv2.namedWindow("Trackbars1")
    cv2.resizeWindow("Trackbars1", 360, 240)
    cv2.createTrackbar("Width Top1", "Trackbars1", intialTracbarVals[0],wT, nothing)
    cv2.createTrackbar("Width Top2", "Trackbars1", intialTracbarVals[1], wT, nothing)
    
    cv2.createTrackbar("Height Top", "Trackbars1", intialTracbarVals[2],hT, nothing)
    
    cv2.createTrackbar("Height Bottom", "Trackbars1", intialTracbarVals[3],hT, nothing)
    


def valTrackbars1():
    widthTop1 = cv2.getTrackbarPos("Width Top1", "Trackbars1")
    widthTop2 = cv2.getTrackbarPos("Width Top2", "Trackbars1")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars1")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars1")

    points = np.float32([(widthTop1, heightTop), (widthTop2, heightTop),
                      (widthTop1 , heightBottom ), (widthTop2, heightBottom)])
    return points
################
def initializeTrackbars(intialTracbarVals,wT=580, hT=300):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)
    cv2.createTrackbar("Lane Select", "Trackbars", 0,3, lambda x: None)
def valTrackbars(wT=580, hT=300):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    return points

def getLaneSelection():
    return cv2.getTrackbarPos("Lane Select", "Trackbars")
def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img
 
def getHistogram(img,minPer=0.1,display= False,region=1):
 
    if region ==1:
        histValues = np.sum(img, axis=0)
    
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
 
    #print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer*maxValue
 
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    #print(basePoint)
 
    if display:
        imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
        for x,intensity in enumerate(histValues):
            if intensity > minValue:color=(255,0,255)
            else: color=(0,0,255)
            cv2.line(imgHist,(x,img.shape[0]),(x,int(img.shape[0]-(intensity//255//region))),color,1)
           
            #cv2.line(imgHist,(x,img.shape[0]),(x,img.shape[0]-intensity//255//region),(255,0,255),1)
        #cv2.circle(imgHist,(basePoint,img.shape[0]),20,(0,255,255),cv2.FILLED)
        return basePoint,imgHist
 
    return basePoint
 
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver