#!/usr/bin/env python

import cv2
import numpy as np

font=cv2.FONT_HERSHEY_SIMPLEX
# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = np.sqrt((372.52-327.5)**2 + (276.981598-203)**2)/np.sqrt((339.6-241.6)**2 + (175.666-115.4037)**2)
tx= (28-240)/beta
ty= (244-320)/beta

# Function that converts image coord to world coord
# Note: input x corresponds to columns in the image, input y is rows in the image
def IMG2W(x,y):
    O_c=320
    O_r=240
    xc=(y-O_r)/beta
    yc=(x-O_c)/beta
    xw=xc-tx
    yw=yc-ty
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    W=np.dot(R,np.array([[xw],[yw]]))
    return W


def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img):
    squares = []
    # img = cv2.GaussianBlur(img, (3, 3), 0)   
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bin = cv2.Canny(img, 30, 100, apertureSize=3)    
    print(cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))    
    contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("outline amounts: %d" % len(contours))
    index = 0
    # outline traversal
    for cnt in contours:
        cnt_len = cv2.arcLength(cnt, True) #calculate outline perimeter
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True) #polygon approch
        # whether four sides, area > 100, convex
        if len(cnt) == 4 and cv2.contourArea(cnt) > 100 and cv2.isContourConvex(cnt):
            M = cv2.moments(cnt) #calculate moment
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])#center
            
            cnt = cnt.reshape(-1, 2)
            max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in range(4)])
            # only squares(cos90 = 0)
            if max_cos < 0.1:
            # quadrilateral
            #if True:
                index = index + 1
                cv2.putText(img,("#%d"%index),(cx,cy),font,0.7,(255,0,255),2)
                squares.append(cnt)
    return squares, img


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 800

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    if color == "orange":
        lower = (10,100,150)     # orange lower
        upper = (25,255,255)   # orange upper
    

    if color == "yellow":
        lower = (20,100,120)     # yellow lower
        upper = (40,255,255)   # yellow upper

    if color == "green":
        lower = (50,100,80)     # green lower
        upper = (80,255,255)   # green upper


    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    print(np.shape(mask_image))

    # ========================= Student's code ends here ===========================

    # keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    # blob_image_center = []
    # num_blobs = len(keypoints)
    # for i in range(num_blobs):
    #     blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    squares, img = find_squares(mask_image)

    # Draw the keypoints on the detected block
    # im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, mask_image)

    im_with_counters = cv2.drawContours( img, squares, -1, (0, 0, 255), 2 )

    xw_yw = []

    # if(num_blobs == 0):
    #     print("No block found!")
    # else:
    #     # Convert image coordinates to global world coordinate using IM2W() function
    #     for i in range(num_blobs):
    #         xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
    # print(blob_image_center)


    # cv2.namedWindow("Camera View")
    # cv2.imshow("Camera View", image_raw)
    # cv2.namedWindow("Mask View")
    # cv2.imshow("Mask View", mask_image)
    # cv2.namedWindow("Keypoint View")
    # cv2.imshow("Keypoint View", im_with_keypoints)
    
    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Counter View")
    cv2.imshow("Counter View", im_with_counters)
    

    cv2.waitKey(2)

    return xw_yw
