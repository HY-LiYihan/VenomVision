# VenomVision

## 项目概述
VenomVision 是一个综合性的视觉处理与控制项目，主要用于机器人视觉应用，尤其是在识别特定目标（如灯光、装甲板）以及控制舵机运动方面。该项目结合了计算机视觉技术（如 OpenCV）、串口通信和相机控制，旨在实现对目标的准确识别和相应的机械控制。

## 功能模块
### 1. 图像采集与处理
- **相机控制**：使用 `hikvision_capture.py` 中的 `read` 方法从相机获取图像帧，并进行格式转换。
- **图像预处理**：`Detector` 类负责对图像进行预处理，包括灰度转换、二值化等操作。

### 2. 目标识别
- **灯光识别**：`Detector` 类中的 `find_lights` 方法用于在二值化图像中查找灯光目标，并通过 `is_light` 方法判断是否为有效灯光。
- **装甲板匹配**：`Detector` 类中的 `matchLights` 方法将识别到的灯光进行配对，通过 `isArmor` 方法判断是否构成有效装甲板，并确定其类型（小或大）。

### 3. 舵机控制
- **舵机管理**：`GimbalServo` 类用于表示和控制单个舵机，通过 `UartServoManager` 进行通信。
- **位置控制**：`GimbalServo` 类的 `set_deviation` 方法根据偏差值设置舵机位置，`center` 方法将舵机移动到中间位置。

### 4. 数据包处理
- **数据包解析**：`Packet` 类负责数据包的打包和解包操作，包括计算校验和、验证数据包合法性等。
- **数据缓冲**：`PacketBuffer` 类作为数据包的中转站，处理接收到的字节流，将合法的数据包添加到队列中。

## 代码结构
```
VenomVision/
├── src/
│   ├── pans/
│   │   ├── packet.py         # 数据包处理
│   │   ├── packet_buffer.py  # 数据包缓冲
│   │   └── uart_servo.py     # 串口舵机控制
│   ├── MvImport/
│   │   ├── CameraParams_header.py  # 相机参数结构体定义
│   │   └── MvCameraControl_class.py  # 相机控制类
│   └── hikvision_capture.py  # 海康威视相机图像采集
├── try_use_mvs_source.py     # 主要测试脚本，包含目标识别和舵机控制逻辑
└── trytry.py                 # 定义装甲板类型和装甲板类
```

## 主要类和方法说明
### 1. `Light` 类
- **功能**：表示识别到的灯光目标，存储灯光的位置、尺寸、倾斜角度和颜色等信息。
- **关键属性**：`center`、`length`、`width`、`tilt_angle`、`color`

### 2. `Detector` 类
- **功能**：负责图像预处理、灯光识别和装甲板匹配。
- **关键方法**：
  - `preprocess_image`：对 RGB 图像进行预处理，包括灰度转换和二值化。
  - `find_lights`：在二值化图像中查找灯光目标。
  - `is_light`：判断是否为有效灯光。
  - `matchLights`：将识别到的灯光进行配对，判断是否构成有效装甲板。
  - `isArmor`：判断两个灯光是否构成有效装甲板，并确定其类型。

### 3. `GimbalServo` 类
- **功能**：表示和控制单个舵机，处理位置计算和角度限制。
- **关键方法**：
  - `set_deviation`：根据偏差值设置舵机位置。
  - `center`：将舵机移动到中间位置。
  - `get_last_deviation`：返回最后一次成功设置的偏差值。

### 4. `Packet` 类
- **功能**：负责数据包的打包和解包操作，包括计算校验和、验证数据包合法性等。
- **关键方法**：
  - `calc_checksum_request`：计算请求包的校验和。
  - `calc_checksum_response`：计算响应包的校验和。
  - `is_response_legal`：验证响应包是否合法。
  - `pack`：将数据打包为二进制数据包。
  - `unpack`：将二进制数据包解包为所需参数。

### 5. `PacketBuffer` 类
- **功能**：作为数据包的中转站，处理接收到的字节流，将合法的数据包添加到队列中。
- **关键方法**：
  - `update`：将新的字节添加到数据包中转站。
  - `has_valid_packet`：判断是否有有效的数据包。
  - `get_packet`：获取队首的数据包。

## 使用示例
### 图像采集与处理
```python
from src.hikvision_capture import HikvisionCapture

# 初始化相机
camera = HikvisionCapture()
camera.start()

# 读取图像帧
ret, frame = camera.read()
if ret:
    # 处理图像帧
    pass

# 停止相机
camera.stop()
```

### 目标识别
```python
from try_use_mvs_source import Detector

# 初始化检测器
detector = Detector()

# 预处理图像
binary_img = detector.preprocess_image(frame)

# 查找灯光
lights = detector.find_lights(binary_img, frame)

# 匹配装甲板
armors = detector.matchLights(lights)
```

### 舵机控制
```python
from src.pans_control import GimbalServo, UartServoManager

# 初始化串口舵机管理器
uservo_manager = UartServoManager()

# 初始化舵机
servo = GimbalServo(servo_id=1, mid_pos=500, min_dev=-100, max_dev=100, uservo_manager=uservo_manager)

# 设置舵机偏差
servo.set_deviation(50)

# 将舵机移动到中间位置
servo.center()
```

## 注意事项
- 确保相机和舵机的硬件连接正常，并且相应的驱动程序已正确安装。
- 在使用串口通信时，注意波特率、数据位、停止位等参数的设置，确保与硬件设备一致。
- 在处理图像时，根据实际情况调整二值化阈值、最小比例、最大比例等参数，以获得更好的识别效果。

## 贡献与反馈
如果您在使用过程中遇到问题或有任何建议，请随时在项目的 GitHub 仓库中提交 Issue 或 Pull Request。我们欢迎社区的贡献和反馈，共同完善 VenomVision 项目。
