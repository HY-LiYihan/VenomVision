# src/Detect/light.py
import numpy as np
import cv2

class Light:
    def __init__(self, box):
        # 存储传入的旋转矩形
        self.box = box
        # 获取旋转矩形的四个顶点
        points = cv2.boxPoints(box)
        points = np.int0(points)
        # 按 y 坐标排序
        sorted_points = sorted(points, key=lambda p: p[1])
        # 计算顶部和底部的中心点
        self.top = (sorted_points[0] + sorted_points[1]) / 2
        self.bottom = (sorted_points[2] + sorted_points[3]) / 2
        self.center_x, self.center_y = int(box[0][0]), int(box[0][1])
        self.center = (self.center_x, self.center_y)
        # 计算长度和宽度
        # self.width, self.length = box[1]
        self.length = np.linalg.norm(self.top - self.bottom)
        self.width = np.linalg.norm(sorted_points[0] - sorted_points[1])
        # 计算倾斜角度
        # self.tilt_angle = box[2]
        self.tilt_angle = np.arctan2(np.abs(self.top[0] - self.bottom[0]), np.abs(self.top[1] - self.bottom[1]))
        self.tilt_angle = self.tilt_angle / np.pi * 180
        # 初始化颜色
        self.color = None
        self.is_light = False