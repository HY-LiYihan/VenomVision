# sample/3test_find_lights_with_pid_control.py
# -- coding: utf-8 --

import sys
import os
# 获取项目根目录的绝对路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将项目根目录添加到 Python 的模块搜索路径中
sys.path.append(project_root)
import time
import serial
import cv2
import numpy as np
from src.pid import PID  # 假设PID类文件与当前脚本同目录
from src.hikvision_capture import HikvisionCamera, HikvisionCameraError
from src.Detect.detector import Detector
from src.GimbalServo.uart_servo import UartServoManager
from src.gimbal_servo_lib import GimbalServo

# --- 舵机配置 ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # 请根据实际端口修改
SERVO_BAUDRATE = 115200
YAW_SERVO_ID = 1
YAW_MID_POS = 2047
YAW_MIN_DEV = -1400
YAW_MAX_DEV = 1400

# --- PID参数（需根据实际调试调整） ---
KP = 0.05     # 比例增益
KI = 0.000      # 积分增益
KD = 0.008       # 微分增益

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

def main():
    # --- 初始化舵机系统 ---
    try:
        uart = serial.Serial(
            port=SERVO_PORT_NAME,
            baudrate=SERVO_BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0.1
        )
        uservo = UartServoManager(uart, servo_id_list=[YAW_SERVO_ID])
        yaw_servo = GimbalServo(
            servo_id=YAW_SERVO_ID,
            mid_pos=YAW_MID_POS,
            min_dev=YAW_MIN_DEV,
            max_dev=YAW_MAX_DEV,
            uservo_manager=uservo
        )
        yaw_servo.center()  # 初始居中
        time.sleep(1.0)     # 等待舵机到位
    except serial.SerialException as e:
        print(f"舵机串口初始化失败: {e}")
        return

    # --- 初始化相机和检测器 ---
    try:
        HikvisionCamera.initialize_sdk()
        detector = Detector(binary_thres=100)
        
        with HikvisionCamera(device_index=0, verbose=False) as cam:
            if not cam.is_open():
                print("相机打开失败")
                return
            
            width, height = cam.get_resolution()
            frame_center_x = width // 2
            frame_center_y = height // 2  # 暂不使用垂直方向
            
            # 初始化PID控制器（控制目标为0，误差=中心x - armor x）
            pid_controller = PID(Kp=KP, Ki=KI, Kd=KD, setpoint=0)
            last_time = time.time()
            
            if not cam.start():
                print("相机数据抓取启动失败")
                return

            print("开始实时控制循环（按'q'退出）")
            error_x = 0
            while True:
                ret, frame_bgr = cam.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                binary_img = detector.preprocess_image(frame_rgb)
                lights = detector.find_lights(binary_img, frame_rgb)
                armors = detector.matchLights(lights)
                
                if armors:
                    # 选择第一个检测到的装甲板（可扩展为多目标处理）
                    armor = armors[0]
                    armor_x, armor_y = armor.center
                    error_x += - frame_center_x + armor_x  # 计算水平偏差

                    current_time = time.time()
                    dt = current_time - last_time
                    dt = dt if dt > 0 else 0.0001  # 防止除零错误
                    # if dt > 0.1:  # 限制PID更新频率
                    #     dt = 0.1
                    last_time = current_time

                    # PID计算输出
                    pid_output = pid_controller.update(error_x, dt)
                    # 限制舵机偏差在安全范围
                    servo_deviation = max(YAW_MIN_DEV, min(YAW_MAX_DEV, pid_output))
                    print(f"PID Output: {pid_output:.2f}, Servo Deviation: {servo_deviation:.2f}, error_x: {error_x:.2f}")
                    yaw_servo.set_deviation(servo_deviation)
                    
                    # 绘制调试信息
                    frame_bgr = draw_armor(frame_bgr, armor)
                    cv2.putText(frame_bgr, f"PID: {pid_output:.2f} Dev:{servo_deviation}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    pid_controller.reset()  # 重置PID控制器
                cv2.imshow("Armor Control", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except HikvisionCameraError as e:
        print(f"相机错误: {e}")
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- 资源释放 ---
        if 'yaw_servo' in locals() and yaw_servo:
            yaw_servo.center()
            time.sleep(0.5)
        if 'uart' in locals() and uart.is_open:
            uart.close()
        
        HikvisionCamera.finalize_sdk()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    main()
