# src/Detect/detector.py
import cv2
import numpy as np
from src.Detect.light import Light
from src.Detect.armor import Armor, ArmorType

class Detector:
    def __init__(self, binary_thres=128, min_ratio=3.5, max_ratio=16.0, max_angle=45.0, min_light_ratio=0.7, max_armor_angle=30.0):
        """
        初始化检测器。
        :param binary_thres: 二值化阈值，默认为 128
        :param min_ratio: 最小比例，默认为 3.5
        :param max_ratio: 最大比例，默认为 14.0
        :param max_angle: 最大倾斜角度，默认为 45.0
        :param min_light_ratio: 最小灯光比例，默认为 0.7
        :param max_armor_angle: 最大装甲板角度，默认为 30.0
        """
        # 初始化参数
        
        self.binary_thres = binary_thres  # 设置二值化的阈值，默认为 128
        self.min_ratio = min_ratio  # 最小比例
        self.max_ratio = max_ratio  # 最大比例
        self.max_angle = max_angle  # 最大倾斜角度
        self.min_light_ratio = min_light_ratio  # 最小灯光比例
        self.max_armor_angle = max_armor_angle  # 最大装甲板角度
        self.debug_lights = []  # 存储调试信息
        self.debug_armors = []  # 存储调试信息
        self.detect_color = 'RED'  # 设置检测颜色
        self.a = {
            'min_light_ratio': 0.7,
           'min_small_center_distance': 0.8,
           'max_small_center_distance': 3.2,
           'min_large_center_distance': 3.2,
           'max_large_center_distance': 5.5,
            'max_angle': 35
        }

    def preprocess_image(self, rgb_img):
        # 将 RGB 图像转换为灰度图像
        gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)

        # 对灰度图像进行二值化处理
        _, binary_img = cv2.threshold(gray_img, self.binary_thres, 255, cv2.THRESH_BINARY)

        # 定义形态学操作的核
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        # 进行形态学开运算去噪声
        denoised_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)

        return denoised_img

    def find_lights(self, binary_img, rgb_img):
        """
        查找二值图像中的连通区域，并返回它们的最小外接矩形的详细参数。

        Args:
            binary_img (numpy.ndarray): 输入的二值图像 (单通道，前景为白色/非零).
            rgb_img (numpy.ndarray): 对应的RGB图像 (用于可视化，可选).

        Returns:
            list: 一个包含字典的列表，每个字典代表一个最小外接矩形，包含以下键：
                - 'center': 矩形中心的坐标 (x, y).
                - 'size': 矩形的尺寸 (width, height).
                - 'angle': 矩形旋转的角度 (浮点数，单位为度，范围通常为 [-90, 0)).
                - 'vertices': 矩形的四个顶点的坐标 (按顺序排列的列表，例如：[top-left, top-right, bottom-right, bottom-left]).
        """
        # 查找轮廓
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lights = []
        for contour in contours:
            # 获取最小外接矩形
            rect = cv2.minAreaRect(contour)
            light = Light(rect)
            if light.size[0] < 10 or light.size[1] < 2:
                continue
            if self.is_light(light):
                # 获取外接矩形
                rect = cv2.boundingRect(contour)
                x, y, w, h = rect
                # 检查矩形是否在图像范围内
                if 0 <= x and 0 <= w and x + w <= rgb_img.shape[1] and 0 <= y and 0 <= h and y + h <= rgb_img.shape[0]:
                    sum_r = 0
                    sum_b = 0
                    # 遍历 ROI
                    for i in range(y, y + h):
                        for j in range(x, x + w):
                            if cv2.pointPolygonTest(contour, (j, i), False) >= 0:
                                # 如果点在轮廓内
                                sum_r += rgb_img[i, j, 2]
                                sum_b += rgb_img[i, j, 0]
                    # 判断颜色
                    light.color = 'RED' if sum_r > sum_b else 'BLUE'
                    if np.abs(sum_b - sum_r) > 200:
                        lights.append(light)
                    
        return lights
        #     # 获取矩形的中心点、尺寸和角度
        #     center, size, angle = rect
        #     center = tuple(map(int, center))
        #     size = tuple(map(int, size))

        #     # 获取矩形的四个顶点
        #     box = cv2.boxPoints(rect)
        #     box = np.intp(box)  # 将浮点数坐标转换为整数
        #     boxes.append(box)

        #     min_rect_info = {
        #         'center': center,
        #         'size': size,
        #         'angle': angle,
        #         'vertices': box.tolist()
        #     }
        #     min_rects_data.append(min_rect_info)



        # return min_rects_data,boxes
        # lights = []
        # for contour in contours:
        #     if len(contour) < 10:
        #         continue
        #     # 计算最小面积矩形
        #     r_rect = cv2.minAreaRect(contour)
        #     light = Light(r_rect)
        #     if self.is_light(light):
        #         # 获取外接矩形
        #         rect = cv2.boundingRect(contour)
        #         x, y, w, h = rect
        #         # 检查矩形是否在图像范围内
        #         if 0 <= x and 0 <= w and x + w <= rgb_img.shape[1] and 0 <= y and 0 <= h and y + h <= rgb_img.shape[0]:
        #             sum_r = 0
        #             sum_b = 0
        #             # 遍历 ROI
        #             for i in range(y, y + h):
        #                 for j in range(x, x + w):
        #                     if cv2.pointPolygonTest(contour, (j, i), False) >= 0:
        #                         # 如果点在轮廓内
        #                         sum_r += rgb_img[i, j, 2]
        #                         sum_b += rgb_img[i, j, 0]
        #             # 判断颜色
        #             light.color = 'RED' if sum_r > sum_b else 'BLUE'
        #             if np.abs(sum_b - sum_r) > 200:
        #                 lights.append(light)

        # return lights

    def is_light(self, light):
        # 判断是否为灯光
        ratio = light.size[0] / light.size[1] if light.size[1] != 0 else 0
        ratio_ok = self.min_ratio < ratio < self.max_ratio
        angle_ok = abs(light.angle) >= 90.0 - self.max_angle
        is_light = ratio_ok and angle_ok

        # 填充调试信息
        light_data = {
            "center_x": light.center[0],
            "center_y": light.center[1],
            "ratio": ratio,
            "angle": light.angle,
            "is_light": is_light
        }
        self.debug_lights.append(light_data)

        # 更新 Light 对象的属性
        light.is_light = is_light
        # if not is_light:
        #     # print(light_data, f'ratio_ok:{ratio_ok},angle_ok:{angle_ok}')

        return is_light

    def matchLights(self, lights):
        armors = []

        # Loop all the pairing of lights
        for i in range(len(lights)):
            for j in range(i + 1, len(lights)):
                # print(f"Matching lights {i} and {j}")
                light_1 = lights[i]
                light_2 = lights[j]
                if light_1.color != self.detect_color or light_2.color != self.detect_color:
                    continue

                # 假设 containLight 函数已经定义
                if self.containLight(light_1, light_2, lights):
                    # print(f"Lights {i} and {j} contain, skipping")
                    continue

                # 假设 isArmor 函数已经定义
                type_ = self.isArmor(light_1, light_2)
                # print(f"Armor type: {type_}")
                if type_ != ArmorType.INVALID:
                    armor = Armor(light_1, light_2)
                    armor.type = type_
                    armors.append(armor)

        return armors

    def containLight(self, light_1, light_2, lights):
        points = [light_1.top, light_1.bottom, light_2.top, light_2.bottom]
        points = np.array(points, dtype=np.int32)

        # 计算边界矩形
        x, y, w, h = cv2.boundingRect(points)

        # 计算矩形的顶点坐标
        box = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.int32)
        for test_light in lights:
            if test_light == light_1 or test_light == light_2:
                continue

            if cv2.pointPolygonTest(box, test_light.top, False) >= 0 or cv2.pointPolygonTest(box, test_light.bottom, False) >= 0:
                return True

        return False

    def isArmor(self, light_1, light_2):
        # Ratio of the length of 2 lights (short side / long side)
        light_length_ratio = min(light_1.size[0], light_2.size[0]) / max(light_1.size[0], light_2.size[0])
        light_ratio_ok = light_length_ratio > self.a['min_light_ratio']
        # print(f"Light ratio: {light_length_ratio}, light_ratio_ok: {light_ratio_ok}")

        # Distance between the center of 2 lights (unit : light length)
        avg_light_length = (light_1.size[0] + light_2.size[0]) / 2
        center_distance = cv2.norm(
            cv2.Mat(np.array(light_1.center, dtype=np.float32)) - cv2.Mat(np.array(light_2.center, dtype=np.float32))) / avg_light_length
        center_distance_ok = (self.a['min_small_center_distance'] <= center_distance < self.a['max_small_center_distance']) or \
            (self.a['min_large_center_distance'] <= center_distance < self.a['max_large_center_distance'])

        # Angle of light center connection
        diff = (light_1.center[0] - light_2.center[0], light_1.center[1] - light_2.center[1])
        if diff[0] != 0:
            angle = abs(np.arctan(diff[1] / diff[0])) * 180 / np.pi
        else:
            angle = 90  # 避免除零错误
        angle_ok = angle < self.max_armor_angle

        is_armor = light_ratio_ok and center_distance_ok and angle_ok

        # Judge armor type
        if is_armor:
            type_ = ArmorType.LARGE if center_distance > self.a['min_large_center_distance'] else ArmorType.SMALL
        else:
            # print(f"Armor detection failed: light_ratio_ok: {light_ratio_ok}, center_distance_ok: {center_distance_ok}, angle_ok: {angle_ok}")
            type_ = ArmorType.INVALID

        # Fill in debug information
        armor_data = {
            'type': type_.value,
            'center_x': (light_1.center[0] + light_2.center[0]) / 2,
            'light_ratio': light_length_ratio,
            'center_distance': center_distance,
            'angle': angle
        }
        self.debug_armors.append(armor_data)

        return type_