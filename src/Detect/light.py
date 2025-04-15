import cv2
import numpy as np

class Light:
    def __init__(self, rect):
        # 存储传入的旋转矩形
        self.rect = rect
        self.center = rect[0]
        self.size = rect[1]
        self.angle = rect[2]
        
        # 确保宽度小于高度
        if self.size[0] < self.size[1]:
            self.size = (self.size[1], self.size[0])
            self.angle += 90
        
        # 修正角度在 -90 到 90 之间
        if self.angle >= 90:
            self.angle -= 180
        
        # 计算最小外接矩形的四个角点
        self.box = cv2.boxPoints(rect)
        self.box = np.int0(self.box)  # 转换为整数坐标
        
        # 初始化颜色和状态
        self.color = None
        self.is_light = False
        
        # 计算上边和下边的中点
        self.top = self.calculate_top_bottom('top')
        self.bottom = self.calculate_top_bottom('bottom')
    
    def calculate_top_bottom(self, position):
        # 根据y坐标确定上边和下边的中点
        if position == 'top':
            # 上边：y坐标最小的两个点
            points = sorted(self.box, key=lambda p: p[1])[:2]
        elif position == 'bottom':
            # 下边：y坐标最大的两个点
            points = sorted(self.box, key=lambda p: p[1])[2:]
        
        # 计算中点
        mid_x = (points[0][0] + points[1][0]) / 2
        mid_y = (points[0][1] + points[1][1]) / 2
        return (mid_x, mid_y)

# 示例使用
rect = ((250, 250), (100, 200), 45)  # 假设一个旋转矩形的中心点、大小和角度
light = Light(rect)
print("Top midpoint:", light.top)
print("Bottom midpoint:", light.bottom)
