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



# img = cv2.imread('modifiedMap.pgm')
img = cv2.imread('multiRoom.pgm')

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

bg = thresh 
prepareWindow(bg,"background", True)


kernel = np.ones((3,3),np.uint8)

#提取出大块障碍物
bigObstacle = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,kernel, iterations=10)
# bigObstacle = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE,kernel, iterations=1)

prepareWindow(bigObstacle,"bigObstacle", True)

#留下墙
smallObstacle = cv2.bitwise_not(cv2.bitwise_xor(thresh, bigObstacle))
smoothSmallObstacle = cv2.morphologyEx(smallObstacle, cv2.MORPH_CLOSE,kernel, iterations=5)
# smoothSmallObstacle = cv2.morphologyEx(smallObstacle, cv2.MORPH_CLOSE,kernel, iterations=0)
prepareWindow(smoothSmallObstacle,"smallObstacle", True)


dist_transform = cv2.distanceTransform(smoothSmallObstacle,cv2.DIST_L2 ,3)

#离墙3像素认为是空白
ret, fg = cv2.threshold(dist_transform,3,255,0)
fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,kernel, iterations=50)
# fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,kernel, iterations=10)

prepareWindow(fg,"foreground", True)

fg = np.uint8(fg)
unknown = cv2.subtract(bg, fg)
prepareWindow(unknown,"unknown",True)

fg[bigObstacle==0] = 0

prepareWindow(fg,"before group seeds",True)

ret, markers = cv2.connectedComponents(fg, 4)
markers= markers+1

# 将未知区域降为最低。 将墙围起来
markers[unknown == 255] = 0
markers[bg == 0] = 1

prepareWindow(markers,"marker1", True)

#分水岭
result = cv2.watershed(img,markers)


prepareWindow(result,"result")

plt.show()