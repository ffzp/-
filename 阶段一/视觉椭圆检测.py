import cv2
import numpy as np
from matplotlib import pyplot as plt
import math

img = cv2.imread("2.jpg", 3)
# img=cv2.blur(img,(1,1))
img0 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
dst = cv2.equalizeHist(img0)  # 直方图均匀化提高图片对比度
imgray = cv2.Canny(dst, 200, 100, 3)  # Canny边缘检测
# cv2. namedWindow("0", 0)
# cv2.imshow("0", imgray)
ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY)  # 图像二值化
# cv2.imshow("1", thresh)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # contours为轮廓集，可以计算轮廓的长度、面积等
for cnt in contours:
    if len(cnt) > 30:
        S1 = cv2.contourArea(cnt)
        ell = cv2.fitEllipse(cnt)
        S2 = math.pi * ell[1][0] * ell[1][1]
        if (S1 / S2) > 0.22:  # 面积比例，可以更改，根据数据集
            img = cv2.ellipse(img, ell, (0, 255, 0), 2)
            print(str(S1) + "    " + str(S2) + "   " + str(ell[0][0]) + "   " + str(ell[0][1]))
            # ell[0][0]为中心横坐标，ell[0][1]为纵坐标
cv2.namedWindow("W1", 0)
cv2.imshow("W1", img)
cv2.waitKey(0)
