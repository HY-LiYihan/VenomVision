# sample/3test_find_lights.py
# -- coding: utf-8 --
import sys
import os

# 获取项目根目录的绝对路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将项目根目录添加到 Python 的模块搜索路径中
sys.path.append(project_root)

from src.hikvision_capture import HikvisionCamera, HikvisionCameraError
from src.Detect.detector import Detector
import cv2
import time
import numpy as np

def draw_armor(image, armor, color=(0, 255, 0), thickness=1):
    """
    在图像上绘制 Armor 结构体
    :param image: 原始图像
    :param armor: Armor 对象
    :param color: 绘制颜色
    :param thickness: 线宽
    :return: 带绘制的图像
    """
    if armor.left_light and armor.right_light:
        # 画左右灯条的矩形框
        left_box = np.int0(armor.left_light.box)
        right_box = np.int0(armor.right_light.box)
        cv2.drawContours(image, [left_box], 0, (0, 0, 255), thickness)
        cv2.drawContours(image, [right_box], 0, (0, 0, 255), thickness)

        # 画中心点
        center = tuple(map(int, armor.center))
        cv2.circle(image, center, 4, color, -1)

        # 显示装甲板编号和置信度
        label = f"{center}"
        cv2.putText(image, label, (center[0] - 50, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 绘制对角线
        left_diagonal_start = tuple(map(int, armor.left_light.top))
        left_diagonal_end = tuple(map(int, armor.right_light.bottom))
        cv2.line(image, left_diagonal_start, left_diagonal_end, color, thickness)

        right_diagonal_start = tuple(map(int, armor.right_light.top))
        right_diagonal_end = tuple(map(int, armor.left_light.bottom))
        cv2.line(image, right_diagonal_start, right_diagonal_end, color, thickness)

    return image

try:
    # 初始化海康威视 SDK
    HikvisionCamera.initialize_sdk()

    # 创建 Detector 实例
    detector = Detector(binary_thres=100)  # 可以调整二值化阈值

    # 使用 with 语句创建海康威视相机对象
    with HikvisionCamera(device_index=0, verbose=False) as cam:
        # 检查相机是否成功打开
        if not cam.is_open():
            print("错误：无法打开相机。")
            sys.exit()

        # 获取相机分辨率
        width, height = cam.get_resolution()
        print(f"相机分辨率: {width}x{height}")

        # 启动相机数据抓取
        if not cam.start():
            print("错误：无法启动数据抓取。")
            sys.exit()

        # --- 主循环 ---
        while True:
            # 获取 BGR 格式的帧
            ret, frame_bgr = cam.read()

            # 检查帧是否抓取成功
            if not ret:
                # 处理帧抓取失败的情况（例如超时）
                time.sleep(0.01)  # 避免忙等待
                continue

            # 将 BGR 图像转换为 RGB 图像，因为 detector 内部处理的是 RGB
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # 预处理图像
            binary_img = detector.preprocess_image(frame_rgb)

            # 查找灯条
            lights = detector.find_lights(binary_img, frame_rgb)

            # 在原始图像上可视化灯条
            frame_with_lights = frame_bgr.copy()
            # for light in lights:
            #     box = light.box.astype(np.int32)
            #             # 可选：在RGB图像上绘制最小外接矩形进行可视化
            #     if frame_with_lights is not None:
            #         cv2.drawContours(frame_with_lights, [box], 0, (0, 0, 255), 2)
            armors = detector.matchLights(lights)
            if armors is not []:
                for armor in armors:
                    draw_armor(frame_with_lights, armor, color=(0, 255, 0), thickness=2)
                    # # 将旋转矩形的四个角点转换为整数坐标
                    # points = armor.left_light.box[0:2]
                    # points.append(armor.right_light.box[0:2])
                    # # 绘制轮廓
                    # cv2.drawContours(frame_with_lights, [points], 0, (255, 0, 0), 2)
                    # # 绘制中心点
                    # center = int(armor.center)
                    # cv2.circle(frame_with_lights, center, 3, (0, 255, 0), -1)  # 绿色
                    # # 标注颜色
                    # cv2.putText(frame_with_lights, armor.color, (center[0] + 5, center[1] + 5),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    # # 标注类型
                    # cv2.putText(frame_with_lights, armor.type.name, (center[0] + 5, center[1] + 20),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    # # 打印装甲板信息
                    # print(f"Armor Type: {armor.type.name}, Color: {armor.color}")
                    # # 打印灯条信息
                    # print(f"Light Center: {light.center}, Light Size: {light.size}, Light Angle: {light.angle}")
                    # # 打印装甲板信息
                    # print(f"Armor Center: {armor.center}, Armor Size: {armor.size}, Armor Angle: {armor.angle}")
                    # # 打印匹配的灯条信息
                    # print(f"Matched Lights: {armor.lights}")
                    # cv2.circle(rgb_img, center, 3, (0, 255, 0), -1) # 绘制中心点
            # for light in lights:
            #     # 将旋转矩形的四个角点转换为整数坐标
            #     points = cv2.boxPoints(light.rect).astype(np.int32)
            #     # 绘制轮廓
            #     cv2.drawContours(frame_with_lights, [points], 0, (0, 255, 255), 2)  # 黄色
            #     # 绘制中心点
            #     center = (int(light.center_x), int(light.center_y))
            #     cv2.circle(frame_with_lights, center, 3, (0, 255, 0), -1)  # 绿色
            #     # 标注颜色
            #     cv2.putText(frame_with_lights, light.color, (center[0] + 5, center[1] + 5),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1) # 紫色

            cv2.imshow("Light Detection", frame_with_lights)
            key = cv2.waitKey(1) & 0xFF
            # 按 'q' 键退出循环
            if key == ord('q'):
                break

    # 'with' 语句退出时会自动调用 cam.release()

except HikvisionCameraError as e:
    print(f"相机错误: {e}")
except Exception as e:
    print(f"发生错误: {e}")
finally:
    # 结束海康威视 SDK
    HikvisionCamera.finalize_sdk()
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()
    print("应用程序结束。")
    
import cv2
import numpy as np

