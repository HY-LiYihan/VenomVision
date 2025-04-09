import cv2
import numpy as np
from enum import Enum
import serial
import time
import sys
from src.uart_servo import UartServoManager
from src.data_table import *

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        """
        初始化 PID 控制器.
        
        参数:
        Kp: 比例增益
        Ki: 积分增益
        Kd: 微分增益
        setpoint: 目标值 (默认是 0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        
        self.previous_error = 0
        self.integral = 0

    def update(self, feedback_value, dt):
        """
        更新 PID 控制器并计算输出.
        
        参数:
        feedback_value: 当前反馈值
        dt: 时间间隔
        
        返回:
        控制器输出
        """
        # 计算误差
        error = self.setpoint - feedback_value
        
        # 计算积分部分
        self.integral += error * dt
        
        # 计算微分部分
        derivative = (error - self.previous_error) / dt
        
        # 计算 PID 输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 保存误差以备下次计算微分部分
        self.previous_error = error
        
        return output

# 参数配置
SERVO_PORT_NAME =  '/dev/ttyUSB0' 	# 舵机串口号
SERVO_BAUDRATE = 115200 	# 舵机的波特率
SERVO_ID = [1,2]				# 舵机ID 
mid = [0,2047,2047]
angle_range = [(0,0),(-1400,1400),(-650,1024)]
# 初始化串口
uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
					 parity=serial.PARITY_NONE, stopbits=1,\
					 bytesize=8,timeout=0)
# 创建舵机对象
uservo = UartServoManager(uart, servo_id_list=SERVO_ID)


def set_angle(id,angle):
    if (angle+mid[id]) >= (angle_range[id][1]+mid[id]):
        uservo.set_position(id, angle_range[id][1]+mid[id])
    elif (angle+mid[id]) <= (angle_range[id][0]+mid[id]):
        uservo.set_position(id, angle_range[id][0]+mid[id])
    else:
        uservo.set_position(id, angle+mid[id])



binary_thres = 200  # 设置二值化的阈值，默认为128

max_ratio = 0.6  # 最大比例
min_ratio = 0.1  # 最小比例
max_angle = 45  # 最大倾斜角度
min_light_ratio = 0.7  # 最小比例


class ArmorType(Enum):
    SMALL = "small"
    LARGE = "large"
    INVALID = "invalid"


class Armor:
    def __init__(self, l1=None, l2=None):
        if l1 and l2:
            if l1.center[0] < l2.center[0]:
                self.left_light = l1
                self.right_light = l2
            else:
                self.left_light = l2
                self.right_light = l1
            self.center = ((np.array(self.left_light.center) + np.array(self.right_light.center)) / 2).tolist()
        else:
            self.left_light = None
            self.right_light = None
            self.center = None
        self.type = ArmorType.INVALID
        self.number_img = None
        self.number = ""
        self.confidence = 0.0
        self.classfication_result = ""


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


class Detector:
    def __init__(self, binary_thres=128, min_ratio=0.5, max_ratio=2.0, max_angle=45.0):
        self.binary_thres = binary_thres  # 设置二值化的阈值，默认为128
        self.min_ratio = min_ratio  # 最小比例
        self.max_ratio = max_ratio  # 最大比例
        self.max_angle = max_angle  # 最大倾斜角度
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

        # 将高于 230 的像素置为 0
        # binary_img[gray_img > 240] = 0

        return binary_img

    def find_lights(self, binary_img, rgb_img):
        # 查找轮廓
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lights = []
        for contour in contours:
            if len(contour) < 5:
                continue

            # 计算最小面积矩形
            r_rect = cv2.minAreaRect(contour)
            light = Light(r_rect)
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
                    if np.abs(sum_b-sum_r)>200:
                        lights.append(light)

        return lights

    def is_light(self, light):
        # 判断是否为灯光
        ratio = light.width / light.length if light.length != 0 else 0
        ratio_ok = self.min_ratio < ratio < self.max_ratio
        angle_ok = abs(light.tilt_angle) < self.max_angle
        is_light = ratio_ok and angle_ok

        # 填充调试信息
        light_data = {
            "center_x": light.center_x,
            "center_y": light.center_y,
            "ratio": ratio,
            "angle": light.tilt_angle,
            "is_light": is_light
        }
        self.debug_lights.append(light_data)

        # 更新Light对象的属性
        light.is_light = is_light
        if not is_light:
            print(light_data, f'ratio_ok:{ratio_ok},angle_ok:{angle_ok}')

        return is_light

    def matchLights(self, lights):
        armors = []

        # Loop all the pairing of lights
        for i in range(len(lights)):
            for j in range(i + 1, len(lights)):
                light_1 = lights[i]
                light_2 = lights[j]
                if light_1.color != self.detect_color or light_2.color != self.detect_color:
                    continue

                # 假设 containLight 函数已经定义
                if self.containLight(light_1, light_2, lights):
                    continue

                # 假设 isArmor 函数已经定义
                type_ = self.isArmor(light_1, light_2)
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
        light_length_ratio = min(light_1.length, light_2.length) / max(light_1.length, light_2.length)
        light_ratio_ok = light_length_ratio > self.a['min_light_ratio']

        # Distance between the center of 2 lights (unit : light length)
        avg_light_length = (light_1.length + light_2.length) / 2
        center_distance = cv2.norm(cv2.Mat(np.array(light_1.center, dtype=np.float32)) - cv2.Mat(np.array(light_2.center, dtype=np.float32))) / avg_light_length
        center_distance_ok = (self.a['min_small_center_distance'] <= center_distance < self.a['max_small_center_distance']) or \
                             (self.a['min_large_center_distance'] <= center_distance < self.a['max_large_center_distance'])

        # Angle of light center connection
        diff = (light_1.center[0] - light_2.center[0], light_1.center[1] - light_2.center[1])
        if diff[0] != 0:
            angle = abs(np.arctan(diff[1] / diff[0])) * 180 / np.pi
        else:
            angle = 90  # 避免除零错误
        angle_ok = angle < self.a['max_angle']

        is_armor = light_ratio_ok and center_distance_ok and angle_ok

        # Judge armor type
        if is_armor:
            type_ = ArmorType.LARGE if center_distance > self.a['min_large_center_distance'] else ArmorType.SMALL
        else:
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


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # 设置像素格式为 MJPG
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 设置宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 设置高度
    cap.set(cv2.CAP_PROP_FPS, 180)           # 设置帧率
    detector = Detector(binary_thres=binary_thres, min_ratio=min_ratio, max_ratio=max_ratio, max_angle=max_angle)
    angle1,angle2=0,969
    set_angle(1,angle1)
    set_angle(2,angle2)
    time.sleep(0.5)
    p = 0.1
    i = 0.0
    d = 0.01
    pid1 = PID(p , i, d)
    pid2 = PID(p , i, d)
    previous_time = time.time()
    while True:
        dots = []
        ret, frame = cap.read()
        if not ret:
            break  # 读取失败时退出循环
        binary_img = detector.preprocess_image(frame)
        lights = detector.find_lights(binary_img, frame)
        armors = detector.matchLights(lights)

        binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        for light in lights:
            cv2.circle(binary_img, (light.center_x, light.center_y), 3, (0, 255, 0), -1)
            # 显示颜色信息
            cv2.putText(binary_img, light.color, (light.center_x, light.center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)

        for armor in armors:
            left_x, left_y = armor.left_light.center
            right_x, right_y = armor.right_light.center
            cv2.line(binary_img, (int(left_x), int(left_y)), (int(right_x), int(right_y)), (0, 0, 255), 2)
            cv2.putText(binary_img, armor.type.value, ((int(left_x) + int(right_x)) // 2, (int(left_y) + int(right_y)) // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        h,w,c = frame.shape
        if len(armors) > 0:
            # 获取当前时间
            current_time = time.time()
            cX = armors[0].center[0]
            cY = armors[0].center[1]
            # 计算时间间隔
            dt = current_time - previous_time
            previous_time = current_time
            angle1 += pid1.update(cX - w//2, dt)
            angle2 += pid2.update(-(cY - h//2), dt)
            set_angle(1,angle1)
            set_angle(2,angle2)
        print(f"{angle1} {angle2}")
        # cv2.imshow('frame', binary_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
