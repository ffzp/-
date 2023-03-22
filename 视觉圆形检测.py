from email.mime import image

import cv2 as cv
import numpy as np


def detect_circles(image):
    dst = cv.pyrMeanShiftFiltering(image, 10, 100)  # 均值偏移滤波
    # cv.imshow("滤波结果", dst)
    dst1 = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)  # 灰度图
    circles = cv.HoughCircles(dst1, cv.HOUGH_GRADIENT, 1, 50, param1=100, param2=100, minRadius=0)  # 霍夫圆检测
    if circles is not None:
        circles = np.uint16(np.around(circles))  # 取整
        for i in circles[0, :]:
            cv.circle(image, (i[0], i[1]), i[2], (0, 0, 255), 2)  # 在原图上画圆
            # cv.circle(image, (i[0], i[1]), 2, (255, 0, 0), 2)  # 在原图上画圆心
        for i in circles[0, :]:
            x = int(i[0])
            y = int(i[1])
            print("圆心坐标为：", (x, y))
        cv.namedWindow("检测结果", 0)
        cv.imshow("检测结果", image)
    else:
        print("未识别到圆")


src = cv.imread("1.jpg")
cv.namedWindow("输入图片", 0)
cv.imshow("输入图片", src)
detect_circles(src)
cv.waitKey(0)
