# sample/2manual_control_servo.py
# -- coding: utf-8 --

import sys
import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np  # 库中可能仍然需要
import os
# 获取项目根目录的绝对路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将项目根目录添加到 Python 的模块搜索路径中
sys.path.append(project_root)
# --- 尝试导入 UartServoManager ---
try:
    from src.GimbalServo.uart_servo import UartServoManager
    print("在主脚本中成功导入 UartServoManager。")
except ImportError as e:
    print(f"在主脚本中导入 UartServoManager 时出错: {e}")
    print("请确保'src/pans/uart_servo.py'存在且可访问。")
    sys.exit("退出: 初始化需要 UartServoManager。")

# --- 尝试导入 GimbalServo ---
try:
    from src.gimbal_servo_lib import GimbalServo
    print("成功导入 GimbalServo 库。")
except ImportError as e:
    print(f"导入 GimbalServo 库时出错: {e}")
    print("请确保'gimbal_servo_lib.py'可访问。")
    sys.exit("退出: 需要 GimbalServo 库。")


# --- 舵机配置（特定于应用程序） ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # <<< 重要: 验证此端口名称是否正确！
SERVO_BAUDRATE = 115200
# 定义此应用程序使用的舵机 ID
# 我们将使用索引 0 表示偏航（ID 1），索引 1 表示俯仰（ID 2）
SERVO_IDS_USED = [1, 2]
# 配置映射: 舵机 ID -> 参数
# 如果 ID 不是连续的或稀疏的，使用字典以提高清晰度
SERVO_CONFIG = {
    1: {  # 偏航舵机配置
        "mid_pos": 2047,
        "min_dev": -1400,
        "max_dev": 1400,
        "label": "偏航"
    },
    2: {  # 俯仰舵机配置
        "mid_pos": 2047,
        "min_dev": -650,
        "max_dev": 1024,
        "label": "俯仰"
    }
    # 如果需要，通过更改 SERVO_IDS_USED
    # 并向 SERVO_CONFIG 添加条目来添加更多舵机
}


# --- 全局变量 ---
uart = None
uservo = None
servo_manager_initialized = False
yaw_servo: GimbalServo | None = None  # 为了清晰的类型提示
pitch_servo: GimbalServo | None = None


# --- 初始化串口和舵机管理器 ---
try:
    print(f"尝试打开串口 {SERVO_PORT_NAME}...")
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0.1)
    # 使用此端口上它可能控制的所有舵机 ID 初始化 UartServoManager
    all_servo_ids_on_port = list(SERVO_CONFIG.keys())  # 如果不同则提供已知列表
    uservo = UartServoManager(uart, servo_id_list=all_servo_ids_on_port)
    print(f"成功打开串口 {SERVO_PORT_NAME}。")
    print(f"为 ID 初始化 UartServoManager: {all_servo_ids_on_port}")
    servo_manager_initialized = True

    # --- 使用库实例化 GimbalServo 对象 ---
    # 检查 ID 1 是否被使用和配置
    if 1 in SERVO_IDS_USED and 1 in SERVO_CONFIG:
        cfg = SERVO_CONFIG[1]
        yaw_servo = GimbalServo(
            servo_id=1,
            mid_pos=cfg["mid_pos"],
            min_dev=cfg["min_dev"],
            max_dev=cfg["max_dev"],
            uservo_manager=uservo  # 传递共享的管理器实例
        )
    else:
        print("警告: 偏航舵机 (ID 1) 未配置或不在 SERVO_IDS_USED 中。")

    # 检查 ID 2 是否被使用和配置
    if 2 in SERVO_IDS_USED and 2 in SERVO_CONFIG:
        cfg = SERVO_CONFIG[2]
        pitch_servo = GimbalServo(
            servo_id=2,
            mid_pos=cfg["mid_pos"],
            min_dev=cfg["min_dev"],
            max_dev=cfg["max_dev"],
            uservo_manager=uservo  # 传递共享的管理器实例
        )
    else:
        print("警告: 俯仰舵机 (ID 2) 未配置或不在 SERVO_IDS_USED 中。")

except serial.SerialException as e:
    print(f"打开串口 {SERVO_PORT_NAME} 时发生致命错误: {e}")
    print("舵机控制将被禁用。检查连接和权限。")
except NameError as e:
    print(f"致命错误: 未找到 UartServoManager 或 GimbalServo 类。检查导入。({e})")
    print("舵机控制将被禁用。")
except Exception as e:
    print(f"在串口/舵机设置期间发生意外错误: {e}")
    import traceback
    traceback.print_exc()
    print("舵机控制将被禁用。")


# --- Tkinter GUI 设置 ---
root = tk.Tk()
root.title("云台控制")
root.geometry("600x560")  # 为了显示状态稍微高一点

# 状态标签
status_text = tk.StringVar()
status_label = ttk.Label(root, textvariable=status_text, foreground="red")
status_label.pack(pady=(5, 0))


# --- 滑块回调函数 ---
def yaw_slider_changed(value):
    """当偏航滑块值改变时调用。"""
    if yaw_servo:
        yaw_servo.set_deviation(float(value))
        yaw_value_label.config(text=f"偏航偏差: {float(value):.1f}")
    else:
        status_text.set("偏航舵机不可用！")


def pitch_slider_changed(value):
    """当俯仰滑块值改变时调用。"""
    if pitch_servo:
        pitch_servo.set_deviation(float(value))
        pitch_value_label.config(text=f"俯仰偏差: {float(value):.1f}")
    else:
        status_text.set("俯仰舵机不可用！")


def center_gimbal():
    """通过滑块将两个舵机都设置到中间位置（偏差为 0）。"""
    if yaw_servo:
        yaw_slider.set(0)  # 让滑块通过回调触发舵机方法
    if pitch_servo:
        pitch_slider.set(0)  # 让滑块通过回调触发舵机方法
    print("正在居中云台（通过滑块触发）。")
    status_text.set("正在居中...")
    root.after(500, lambda: status_text.set(""))  # 延迟后清除状态


# --- 创建 GUI 小部件 ---
# 样式（可选）
style = ttk.Style()
try:
    # 使用可用的主题（例如 'clam'，'alt'，'default'，'classic'）
    style.theme_use('clam')
except tk.TclError:
    print("注意: 'clam' 主题不可用，使用默认主题。")  # 回退
style.configure("TScale", troughcolor='lightblue', sliderlength=25)
style.configure("TButton", padding=6, relief="flat", background="#e0e0e0")
style.configure("Red.TLabel", foreground="red")  # 用于状态

# 如果可用，使用配置中的舵机标签
yaw_cfg = SERVO_CONFIG.get(1, {"label": "偏航", "min_dev": -100, "max_dev": 100})  # 默认回退
pitch_cfg = SERVO_CONFIG.get(2, {"label": "俯仰", "min_dev": -100, "max_dev": 100})

# 偏航滑块（水平）
yaw_widget_label = ttk.Label(root, text=f"{yaw_cfg['label']} 控制 (ID 1)")
yaw_widget_label.pack(pady=(10, 0))
yaw_slider = ttk.Scale(root, from_=yaw_cfg["min_dev"], to=yaw_cfg["max_dev"], orient=tk.HORIZONTAL,
                       length=500, command=yaw_slider_changed, style="TScale")
yaw_slider.pack(pady=(5, 0))
yaw_value_label = ttk.Label(root, text=f"偏航偏差: 0.0")
yaw_value_label.pack()

# 俯仰滑块（垂直）
pitch_widget_label = ttk.Label(root, text=f"{pitch_cfg['label']} 控制 (ID 2)")
pitch_widget_label.pack(pady=(10, 0))
# 反转 from_ 和 to 以实现直观的 GUI 上下控制
pitch_slider = ttk.Scale(root, from_=pitch_cfg["max_dev"], to=pitch_cfg["min_dev"], orient=tk.VERTICAL,
                        length=200, command=pitch_slider_changed, style="TScale")
pitch_slider.pack(pady=(5, 0))
pitch_value_label = ttk.Label(root, text=f"俯仰偏差: 0.0")
pitch_value_label.pack()

# 居中按钮
center_button = ttk.Button(root, text="居中云台", command=center_gimbal, style="TButton")
center_button.pack(pady=15)


# --- 设置初始状态 ---
if servo_manager_initialized and (yaw_servo or pitch_servo):
    status_text.set("舵机已初始化")
    status_label.config(foreground="green")
    if yaw_servo:
        yaw_slider.set(0)
        # yaw_servo.center() # 如果滑块回调最初不触发，则显式居中
    else:
        yaw_slider.config(state=tk.DISABLED)
        yaw_widget_label.config(text=f"{yaw_cfg['label']} 控制 (已禁用)")

    if pitch_servo:
        pitch_slider.set(0)
        # pitch_servo.center() # 如果滑块回调最初不触发，则显式居中
    else:
        pitch_slider.config(state=tk.DISABLED)
        pitch_widget_label.config(text=f"{pitch_cfg['label']} 控制 (已禁用)")

    # 在设置滑块后显式地在启动时居中舵机
    root.after(100, center_gimbal)  # 小延迟以确保 GUI 更新

else:
    # 如果舵机初始化失败或未创建任何舵机，则禁用所有小部件
    status_text.set("舵机错误 - 检查控制台")
    status_label.config(foreground="red")
    yaw_slider.config(state=tk.DISABLED)
    pitch_slider.config(state=tk.DISABLED)
    center_button.config(state=tk.DISABLED)
    yaw_widget_label.config(text=f"{yaw_cfg['label']} 控制 (错误)")
    pitch_widget_label.config(text=f"{pitch_cfg['label']} 控制 (错误)")
    root.title("云台控制 (舵机错误)")


# --- 清理函数 ---
def on_closing():
    """处理窗口关闭事件。"""
    print("正在关闭应用程序...")
    if servo_manager_initialized and uart is not None and uart.is_open:
        print("在关闭前居中舵机...")
        try:
            # 使用舵机对象居中
            if yaw_servo:
                yaw_servo.center()
                time.sleep(0.1)  # 小延迟以发送/处理命令
            if pitch_servo:
                pitch_servo.center()
                time.sleep(0.1)
        except Exception as e:
            print(f"关闭时居中舵机出错: {e}")
        finally:
            print("正在关闭串口...")
            uart.close()
            print("串口已关闭。")
    elif uart and uart.is_open:
        print("正在关闭串口（管理器可能未初始化）...")
        uart.close()
        print("串口已关闭。")
    else:
        print("串口未打开或未初始化。")
    root.destroy()  # 关闭 Tkinter 窗口

# 将关闭函数绑定到窗口关闭按钮
root.protocol("WM_DELETE_WINDOW", on_closing)

# --- 启动 Tkinter 主循环 ---
print("正在启动 GUI。关闭窗口或在终端中按 Ctrl+C 退出。")
try:
    root.mainloop()
except KeyboardInterrupt:
    print("\n检测到键盘中断。")
    on_closing()
