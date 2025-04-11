# sample/4test_detect_preprosess.py
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

try:
    # 初始化海康威视 SDK
    HikvisionCamera.initialize_sdk()

    # 创建 Detector 实例
    detector = Detector()

    # 使用 with 语句创建海康威视相机对象
    with HikvisionCamera(device_index=0, verbose=False) as cam:  # 使用 verbose=True 可查看调试信息
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

            # 将 BGR 转换为 RGB
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # 调用 preprocess_image 函数进行处理
            binary_img = detector.preprocess_image(frame_rgb)

            # 将二值化后的图像转换为 BGR 以便显示
            binary_img_bgr = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)

            cv2.imshow("Preprocessed Image", binary_img_bgr)  # 显示二值化后的图像
            cv2.imshow("Raw Image", frame_bgr)  # 显示二值化后的图像
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
    # 关闭串口（如果已打开）
    # if uart is not None and uart.is_open: uart.close()
    print("应用程序结束。")