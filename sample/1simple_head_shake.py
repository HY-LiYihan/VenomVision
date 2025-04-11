# sample/1simple_head_shake.py
# -- coding: utf-8 --

import sys
import os
import serial
import time
# 获取项目根目录的绝对路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将项目根目录添加到 Python 的模块搜索路径中
sys.path.append(project_root)
# --- 配置 ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # <<< 重要: 验证此端口名称！
SERVO_BAUDRATE = 115200
SLEEP_DURATION = 1.5  # 每次移动之间的间隔时间，单位为秒

# --- 舵机特定配置（偏航舵机） ---
YAW_SERVO_ID = 1
YAW_MID_POS = 2047
YAW_MIN_DEV = -1400
YAW_MAX_DEV = 1400

# --- 尝试导入所需的类 ---
try:
    # 初始化通信通道所需
    from src.GimbalServo.uart_servo import UartServoManager
    # 我们创建的库类
    from src.gimbal_servo_lib import GimbalServo
    print("库已成功导入。")
except ImportError as e:
    print(f"导入所需模块时出错: {e}")
    print("请确保 'gimbal_servo_lib.py' 和'src/pans/uart_servo.py' 可访问。")
    sys.exit("因导入错误而退出。")

# --- 全局变量 ---
uart = None
uservo = None
yaw_servo = None

# --- 主执行部分 ---
try:
    # --- 初始化串口和舵机管理器 ---
    print(f"尝试打开串口 {SERVO_PORT_NAME}...")
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0.1)

    # 仅使用我们打算在此简单脚本中使用的 ID 初始化管理器
    uservo = UartServoManager(uart, servo_id_list=[YAW_SERVO_ID])
    print(f"串口已打开，并且已为 ID: {YAW_SERVO_ID} 初始化 UartServoManager。")

    # --- 使用库初始化特定的舵机 ---
    yaw_servo = GimbalServo(
        servo_id=YAW_SERVO_ID,
        mid_pos=YAW_MID_POS,
        min_dev=YAW_MIN_DEV,
        max_dev=YAW_MAX_DEV,
        uservo_manager=uservo  # 传递管理器实例
    )
    print("偏航舵机对象已创建。")

    # --- 初始居中 ---
    print("初始居中舵机...")
    yaw_servo.center()
    time.sleep(1.0)  # 给居中操作一些时间

    # --- 移动循环 ---
    print("\n开始摇头循环（按 Ctrl+C 退出）...")
    while True:
        # 移动到最大偏差（例如，向右）
        print(f"移动到最大偏差的四分之一 ({YAW_MAX_DEV/4})...")
        yaw_servo.set_deviation(YAW_MAX_DEV/4)
        time.sleep(SLEEP_DURATION)

        # 移动到最小偏差（例如，向左）
        print(f"移动到最小偏差的四分之一 ({YAW_MIN_DEV/4})...")
        yaw_servo.set_deviation(YAW_MIN_DEV/4)
        time.sleep(SLEEP_DURATION)

except serial.SerialException as e:
    print(f"\n错误: 无法打开或在串口 {SERVO_PORT_NAME} 上通信。")
    print(f"串口错误: {e}")
    print("请检查连接、权限和端口名称。")
except KeyboardInterrupt:
    print("\n检测到 Ctrl+C。停止。")
except Exception as e:
    print(f"\n发生了意外错误: {e}")
    import traceback
    traceback.print_exc()

finally:
    # --- 清理 ---
    print("\n开始清理...")
    if yaw_servo:  # 检查舵机对象是否成功创建
        print("退出前居中舵机...")
        try:
            yaw_servo.center()
            time.sleep(0.5)  # 给命令执行一些时间
        except Exception as e:
            print(f"最终居中时出错: {e}")

    if uart and uart.is_open:
        print("关闭串口...")
        uart.close()
        print("串口已关闭。")
    else:
        print("串口未打开或未初始化。")

    print("程序已完成。")
    sys.exit(0)  # 显式退出
