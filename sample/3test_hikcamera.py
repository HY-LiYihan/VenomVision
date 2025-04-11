# sample/3test_hikcamera.py
# -- coding: utf-8 --
import sys
import os

# 获取项目根目录的绝对路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将项目根目录添加到 Python 的模块搜索路径中
sys.path.append(project_root)

from src.hikvision_capture import HikvisionCamera, HikvisionCameraError
# ... 脚本的其余部分 ...
import sys
import cv2
import time

try:
    # 初始化海康威视 SDK
    HikvisionCamera.initialize_sdk()

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

            cv2.imshow("Visualization", frame_bgr)  # 显示原始视频流或处理后的图像
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
