import cv2
import cv2.aruco as aruco
import numpy as np
import os
def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

def arucoAug(bbox, id, img, imgAug, drawId = True):
    tl = bbox[0][0][0], bbox[0][0][1]
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]
    h, w, c = imgAug.shape
    pts1 = np.array([tl, tr, br, bl])
    pts2 = np.float32([[0,0], [w,0], [w,h], [0,h]])
    matrix, _ = cv2.findHomography(pts2, pts1)
    imgout = cv2.warpPerspective(imgAug, matrix, (img.shape[1], img.shape[0]))
    return imgout
cap = cv2.VideoCapture(0)
imgAug = cv2.imread(r"/Users/iyongbin/local/test_aruco/image/mango.jpeg")
while True:
    success, img = cap.read()
    arucofound = findArucoMarkers(img)
     # loop through all the markers and augment each one
    if  len(arucofound[0])!=0:
        for bbox, id in zip(arucofound[0], arucofound[1]):
            img = arucoAug(bbox, id, img, imgAug)
    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
cap.release()
#cv2.destroyAllWindows()
