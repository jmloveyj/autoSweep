#!/usr/local/bin/python
#-*-coding:utf-8-*- 
import numpy as np
import cv2
from matplotlib import pyplot as plt

currentWindowId = 0

def prepareWindow(img, title, ifGray = False):
    global currentWindowId
    currentWindowId = currentWindowId +1
    plt.subplot(3,3,currentWindowId)
    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    if(ifGray):
        plt.imshow(img,'gray')
    else:
        plt.imshow(img)

img = cv2.imread('room2.jpg')

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

bg = thresh 
prepareWindow(bg,"original", True)


kernel = np.ones((3,3),np.uint8)

#去噪音
bigObstacle = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,kernel, iterations=2)
prepareWindow(bigObstacle,"real map", True)
cv2.imwrite('multiRoom.pgm',bigObstacle)

greyBigObstacle = bigObstacle
greyBigObstacle[bigObstacle<256] = 220
prepareWindow(greyBigObstacle,"graph",True)
cv2.imwrite('room_grey.pgm',greyBigObstacle)


plt.show()