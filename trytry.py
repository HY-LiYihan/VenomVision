# -- coding: utf-8 --

import sys
import ctypes
import numpy as np
import cv2
import serial
import time
import threading
from enum import Enum

# --- HIKVISION SDK Import ---
# Adjust the path if your SDK installation differs
try:
    # Assume MVS SDK is in the system path or adjacent directory
    # If not, uncomment and adjust the sys.path.append line
    sys.path.append("/opt/MVS/Samples/64/Python/MvImport")
    from MvCameraControl_class import *
except ImportError as e:
    print(f"Error: MvCameraControl_class not found. {e}")
    print("Please ensure the MVS SDK Python samples path is correct or in PYTHONPATH.")
    sys.exit()

# --- Your Existing Code ---
# Assuming these files are in the same directory or accessible via PYTHONPATH
try:
    from src.uart_servo import UartServoManager
    # Assuming data_table defines necessary servo constants if not hardcoded
    # from src.data_table import *
except ImportError as e:
     print(f"Error importing local modules (uart_servo/data_table): {e}")
     print("Ensure src/uart_servo.py and src/data_table.py exist and are accessible.")
     # If data_table isn't strictly needed and constants are below, you might comment out its import.
     # sys.exit()

# --- PID Class ---
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0
        self._last_output = 0 # Store last output for saturation

    def update(self, feedback_value, dt, out_min=-np.inf, out_max=np.inf):
        if dt <= 0:  # Prevent division by zero or instability
            # Return last output or 0 if no valid dt
            # print("Warning: dt <= 0 in PID update")
            return self._last_output

        error = self.setpoint - feedback_value
        self.integral += error * dt
        # Optional: Anti-windup for integral term
        # self.integral = np.clip(self.integral, integral_min, integral_max)

        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error

        # Clamp output
        output = np.clip(output, out_min, out_max)
        self._last_output = output
        return output

# --- Servo Configuration ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # Check if this is correct
SERVO_BAUDRATE = 115200
SERVO_ID = [1, 2]
MID_POS = [0, 2047, 2047]  # Index 0 unused, using 1 and 2
# Angle range defines *deviation* from MID_POS in servo units
ANGLE_RANGE_DEV = [(0, 0), (-1400, 1400), (-650, 1024)] # (min_deviation, max_deviation)

# --- Initialize Serial and Servo Manager ---
uart = None
uservo = None
try:
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0.1) # Slightly increased timeout
    uservo = UartServoManager(uart, servo_id_list=SERVO_ID)
    print(f"Serial port {SERVO_PORT_NAME} opened successfully.")
except serial.SerialException as e:
    print(f"Error opening serial port {SERVO_PORT_NAME}: {e}")
    print("Servo control will be disabled.")
    # Allow script to continue without servos for testing vision?
    # sys.exit() # Or exit if servos are essential
except NameError:
    print("Error: UartServoManager not defined. Check import.")
    sys.exit()

def set_angle(servo_id, angle_deviation):
    """ Sets servo position based on deviation from middle position. """
    if uservo is None:
        # print("Servo manager not initialized, skipping set_angle")
        return

    if servo_id < 1 or servo_id >= len(MID_POS) or servo_id >= len(ANGLE_RANGE_DEV):
        print(f"Error: Invalid servo ID {servo_id}")
        return

    mid_val = MID_POS[servo_id]
    min_dev, max_dev = ANGLE_RANGE_DEV[servo_id]

    # Calculate target position and clamp within allowed absolute range
    target_pos = mid_val + angle_deviation
    min_abs_pos = mid_val + min_dev
    max_abs_pos = mid_val + max_dev

    clamped_pos = int(np.clip(target_pos, min_abs_pos, max_abs_pos))

    try:
        # print(f"Setting Servo {servo_id} to pos: {clamped_pos} (deviation: {angle_deviation})")
        uservo.set_position(servo_id, clamped_pos)
    except Exception as e:
        print(f"Error setting servo {servo_id} position: {e}")


# --- Armor Detection Classes and Parameters ---
binary_thres = 50 # Adjusted based on potential lighting variance
max_ratio = 0.8    # Light ratio limits (tighter?)
min_ratio = 0.1
max_angle = 35     # Light tilt angle limit

class ArmorType(Enum):
    SMALL = "small"
    LARGE = "large"
    INVALID = "invalid"

class Light:
    def __init__(self, box):
        self.box = box
        points = np.int0(cv2.boxPoints(box))
        sorted_points = sorted(points, key=lambda p: p[1])
        self.top = ((sorted_points[0] + sorted_points[1]) / 2).astype(int)
        self.bottom = ((sorted_points[2] + sorted_points[3]) / 2).astype(int)
        self.center = ((self.top + self.bottom) / 2).astype(int)
        self.center_x, self.center_y = self.center[0], self.center[1]
        self.length = np.linalg.norm(self.top - self.bottom)
        self.width = np.linalg.norm(sorted_points[0] - sorted_points[1])
        # Robust angle calculation
        delta_x = self.top[0] - self.bottom[0]
        delta_y = self.top[1] - self.bottom[1]
        self.tilt_angle = np.arctan2(np.abs(delta_x), np.abs(delta_y)) * 180 / np.pi if delta_y != 0 else (90 if delta_x != 0 else 0)
        self.color = None
        self.is_light = False # Added for clarity

class Armor:
    def __init__(self, l1=None, l2=None):
        if l1 and l2:
            if l1.center[0] < l2.center[0]:
                self.left_light = l1
                self.right_light = l2
            else:
                self.left_light = l2
                self.right_light = l1
            self.center = ((np.array(self.left_light.center) + np.array(self.right_light.center)) / 2).astype(int).tolist()
        else: # Should not happen if created correctly
            self.left_light = None
            self.right_light = None
            self.center = None
        self.type = ArmorType.INVALID
        # Number recognition parts removed for brevity, add back if needed
        # self.number_img = None
        # self.number = ""
        # self.confidence = 0.0
        # self.classfication_result = ""

class Detector:
    def __init__(self, binary_thres=128, min_light_ratio=0.1, max_light_ratio=0.8, max_light_angle=35.0):
        self.binary_thres = binary_thres
        self.min_light_ratio = min_light_ratio # Use consistent naming
        self.max_light_ratio = max_light_ratio
        self.max_light_angle = max_light_angle
        self.detect_color = 'BLUE'  # Default or set dynamically 'RED' / 'BLUE'
        self.armor_params = { # Renamed from 'a' for clarity
           'min_light_length_ratio': 0.7, # Ratio between the two lights in an armor pair
           'min_small_center_distance': 0.8, # Distance thresholds (relative to avg light length)
           'max_small_center_distance': 3.2,
           'min_large_center_distance': 3.2,
           'max_large_center_distance': 5.5,
           'max_delta_angle': 35 # Max angle difference between lights center line and horizontal
        }
        # Debug lists cleared each frame now
        # self.debug_lights = []
        # self.debug_armors = []

    def preprocess_image(self, bgr_img):
        # Color Filtering (Example for BLUE detection)
        # Convert to HSV for better color separation
        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        # if self.detect_color == 'BLUE':
        #     # HSV range for blue (adjust these values based on testing)
        #     lower_blue = np.array([90, 100, 100])
        #     upper_blue = np.array([130, 255, 255])
        #     color_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        # elif self.detect_color == 'RED':
        #      # HSV range for red (can be tricky due to wrapping around 0/180)
        #     lower_red1 = np.array([0, 120, 100])
        #     upper_red1 = np.array([15, 255, 255])
        #     lower_red2 = np.array([165, 120, 100])
        #     upper_red2 = np.array([180, 255, 255])
        #     mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        #     mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        #     color_mask = cv2.bitwise_or(mask1, mask2)
        # else: # Default to grayscale threshold if color not specified
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, color_mask = cv2.threshold(gray_img, self.binary_thres, 255, cv2.THRESH_BINARY)
             # print("Warning: detect_color not BLUE or RED, using grayscale threshold.")


        # Optional: Morphological operations to clean up mask
        # kernel = np.ones((3,3), np.uint8)
        # color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
        # color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

        # --- Brightness Threshold (apply to original BGR, masked by color) ---
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        # Apply brightness threshold only where color mask is active
        _, bright_mask = cv2.threshold(gray_img, self.binary_thres, 255, cv2.THRESH_BINARY)

        # Combine color and brightness masks
        binary_img = cv2.bitwise_and(color_mask, bright_mask)

        # Optional: Dilation to connect nearby parts of lights
        # kernel_dilate = np.ones((3, 3), np.uint8)
        # binary_img = cv2.dilate(binary_img, kernel_dilate, iterations=1)

        return binary_img,binary_img# color_mask # Return color mask for is_light check

    def find_lights(self, binary_img, bgr_img, color_mask):
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lights = []
        # self.debug_lights = [] # Clear debug list for this frame

        for contour in contours:
            if len(contour) < 5: # Need at least 5 points for fitEllipse / minAreaRect
                continue

            # Use minAreaRect for potentially rotated lights
            r_rect = cv2.minAreaRect(contour)
            light = Light(r_rect)

            # Check light geometry first (faster)
            if self.is_light_geometry(light):
                 # Check color using the original color mask (more reliable than averaging)
                 # Check if the light center falls within the detected color region
                 if 0 <= light.center_y < color_mask.shape[0] and \
                    0 <= light.center_x < color_mask.shape[1] and \
                    color_mask[light.center_y, light.center_x] > 0:
                     light.color = self.detect_color # Assign detected color
                     light.is_light = True # Mark as valid light
                     lights.append(light)
                 else:
                    # Geometry OK, but color check failed (might happen near edges of mask)
                    # print(f"Light geometry OK but center color check failed at {light.center}")
                    pass
            else:
                 # Geometry check failed
                 # print(f"Light geometry FAILED: ratio={light.width / light.length if light.length else 0:.2f}, angle={light.tilt_angle:.1f}")
                 pass

        return lights

    def is_light_geometry(self, light):
        # Check aspect ratio (width/length)
        if light.length < 1e-6: # Avoid division by zero
            return False
        ratio = light.width / light.length
        ratio_ok = self.min_light_ratio < ratio < self.max_light_ratio

        # Check tilt angle
        angle_ok = abs(light.tilt_angle) < self.max_light_angle

        return ratio_ok and angle_ok

    def match_lights(self, lights):
        armors = []
        # self.debug_armors = [] # Clear debug list

        # Sort lights by x-coordinate for efficient pairing
        lights.sort(key=lambda l: l.center_x)

        for i in range(len(lights)):
            for j in range(i + 1, len(lights)):
                l1 = lights[i]
                l2 = lights[j]

                # Basic check: should have same color (already filtered, but good practice)
                if l1.color != l2.color:
                    continue

                # Check relative geometry to see if they form a valid armor
                armor_type = self.is_armor_pair(l1, l2)
                if armor_type != ArmorType.INVALID:
                    # Optional: Check if a light is contained within the bounding box of this pair
                    # This helps filter out overlapping false positives but adds computation
                    # if self.contain_light(l1, l2, lights):
                    #    continue

                    armor = Armor(l1, l2)
                    armor.type = armor_type
                    armors.append(armor)

        # Optional: Sort armors (e.g., by x-coordinate, size)
        armors.sort(key=lambda a: a.center[0])
        return armors

    def is_armor_pair(self, l1, l2):
        # Ratio of the lengths of the two lights
        light_length_ratio = min(l1.length, l2.length) / max(l1.length, l2.length) if max(l1.length, l2.length)>0 else 0
        light_ratio_ok = light_length_ratio > self.armor_params['min_light_length_ratio']

        # Distance between centers relative to average light length
        avg_light_length = (l1.length + l2.length) / 2
        if avg_light_length < 1e-6: return ArmorType.INVALID # Avoid division by zero

        center_diff = np.array(l1.center) - np.array(l2.center)
        center_distance_val = np.linalg.norm(center_diff)
        center_distance_ratio = center_distance_val / avg_light_length

        # Check if distance falls within small or large armor range
        is_small = (self.armor_params['min_small_center_distance'] <= center_distance_ratio < self.armor_params['max_small_center_distance'])
        is_large = (self.armor_params['min_large_center_distance'] <= center_distance_ratio < self.armor_params['max_large_center_distance'])
        center_distance_ok = is_small or is_large

        # Angle of the line connecting light centers
        delta_angle = abs(np.arctan2(center_diff[1], center_diff[0]) * 180 / np.pi)
        # Correct angle to be difference from horizontal (0 degrees)
        # Angle should be close to 0 for horizontal armors
        angle_ok = delta_angle < self.armor_params['max_delta_angle']

        is_armor = light_ratio_ok and center_distance_ok and angle_ok

        if is_armor:
            armor_type = ArmorType.LARGE if is_large else ArmorType.SMALL
        else:
            # Debug print why it failed
            # print(f"Armor check FAIL: L1({l1.center_x},{l1.center_y}), L2({l2.center_x},{l2.center_y}) | "
            #      f"ratio_ok={light_ratio_ok}({light_length_ratio:.2f}), "
            #      f"dist_ok={center_distance_ok}({center_distance_ratio:.2f}), "
            #      f"angle_ok={angle_ok}({delta_angle:.1f})")
            armor_type = ArmorType.INVALID

        return armor_type

    # Optional: Containment check (can be computationally expensive)
    def contain_light(self, l1, l2, all_lights):
        # Calculate bounding box of the armor pair
        points = np.array([l1.top, l1.bottom, l2.top, l2.bottom], dtype=np.int32)
        x, y, w, h = cv2.boundingRect(points)
        armor_bbox_poly = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.int32)

        for test_light in all_lights:
            if test_light == l1 or test_light == l2:
                continue
            # Check if the center of another light is inside the armor bounding box
            if cv2.pointPolygonTest(armor_bbox_poly, tuple(test_light.center), False) >= 0:
                # print(f"Containment: Light at {test_light.center} is inside armor {l1.center} - {l2.center}")
                return True
        return False

# --- Main Execution ---
def main():
    global uservo, uart # Allow modification in finally block

    # --- Hikvision Camera Setup ---
    cam = None
    h_display_thread = None
    g_exit_flag = False
    width = 0
    height = 0
    p_converted_data = None # Buffer for converted image

    try:
        # Initialize SDK
        MvCamera.MV_CC_Initialize()
        print("MVS SDK Initialized.")

        # Enumerate Devices
        device_list = MV_CC_DEVICE_INFO_LIST()
        tlayer_type = MV_GIGE_DEVICE | MV_USB_DEVICE # Add other types if needed
        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if ret != 0 or device_list.nDeviceNum == 0:
            print(f"Error: Enum devices failed ({ret:#x}) or no devices found.")
            return

        print(f"Found {device_list.nDeviceNum} devices.")
        # Basic device listing (add more details if needed)
        for i in range(device_list.nDeviceNum):
            info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if info.nTLayerType == MV_GIGE_DEVICE:
                 print(f"[{i}] GIGE Device")
            elif info.nTLayerType == MV_USB_DEVICE:
                 print(f"[{i}] USB Device")
            else:
                 print(f"[{i}] Other Device (Type: {info.nTLayerType})")


        # Select Device (simplified: use first found)
        n_connection_num = 0
        if device_list.nDeviceNum > 1:
             try:
                 idx_str = input(f"Select device index [0-{device_list.nDeviceNum-1}]: ")
                 n_connection_num = int(idx_str)
                 if not (0 <= n_connection_num < device_list.nDeviceNum):
                     print("Invalid index, using 0.")
                     n_connection_num = 0
             except ValueError:
                 print("Invalid input, using 0.")
                 n_connection_num = 0
        print(f"Connecting to device index {n_connection_num}...")


        # Create Camera Instance & Handle
        cam = MvCamera()
        st_device_info = cast(device_list.pDeviceInfo[n_connection_num], POINTER(MV_CC_DEVICE_INFO)).contents
        ret = cam.MV_CC_CreateHandle(st_device_info)
        if ret != 0: raise Exception(f"Create handle fail! ret[0x{ret:x}]")

        # Open Device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0: raise Exception(f"Open device fail! ret[0x{ret:x}]")
        print("Camera device opened.")

        # Optimize Packet Size (GigE Only)
        if st_device_info.nTLayerType == MV_GIGE_DEVICE:
            n_packet_size = cam.MV_CC_GetOptimalPacketSize()
            if n_packet_size > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", n_packet_size)
                if ret != 0: print(f"Warning: Set Packet Size fail! ret[0x{ret:x}]")
                else: print(f"Optimal packet size set to {n_packet_size}.")
            else: print(f"Warning: Get Optimal Packet Size fail! ret[0x{n_packet_size:x}]")

        # Set Trigger Mode Off (Continuous)
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0: print(f"Warning: Set trigger mode off fail! ret[0x{ret:x}]")

        # --- Get Camera Resolution ---
        st_width = MVCC_INTVALUE_EX()
        st_height = MVCC_INTVALUE_EX()
        ret = cam.MV_CC_GetIntValueEx("Width", st_width)
        if ret!=0: raise Exception(f"Get Width failed ({ret:#x})")
        ret = cam.MV_CC_GetIntValueEx("Height", st_height)
        if ret!=0: raise Exception(f"Get Height failed ({ret:#x})")
        width = st_width.nCurValue
        height = st_height.nCurValue
        print(f"Camera resolution: {width} x {height}")

        # --- Allocate Conversion Buffer ---
        # Target format: BGR8 for OpenCV
        converted_data_size = width * height * 3
        p_converted_data = (ctypes.c_ubyte * converted_data_size)()


        # --- Start Grabbing ---
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0: raise Exception(f"Start grabbing fail! ret[0x{ret:x}]")
        print("Camera stream started. Press 'q' in OpenCV window to exit.")
        print("Press 'c' to switch target color (RED/BLUE).")


        # --- Initialize Detector, PID, Servos ---
        detector = Detector(binary_thres=binary_thres,
                            min_light_ratio=min_ratio,
                            max_light_ratio=max_ratio,
                            max_light_angle=max_angle)
        # Initial servo positions (deviation from middle)
        angle1_dev = 0.0
        angle2_dev = 0.0 # Start gimbal level
        set_angle(1, angle1_dev) # Yaw
        set_angle(2, angle2_dev) # Pitch
        time.sleep(0.5) # Allow servos to reach initial position

        # PID Controllers (Tune these gains!)
        # Axis 1 (Yaw / Horizontal - Servo ID 1)
        pid1_kp, pid1_ki, pid1_kd = 0.08, 0.0, 0.008 # Initial guess - NEEDS TUNING
        pid1 = PID(pid1_kp, pid1_ki, pid1_kd, setpoint=0) # Target error is 0 (center of screen)
        # Axis 2 (Pitch / Vertical - Servo ID 2)
        pid2_kp, pid2_ki, pid2_kd = 0.08, 0.0, 0.008 # Initial guess - NEEDS TUNING
        pid2 = PID(pid2_kp, pid2_ki, pid2_kd, setpoint=0) # Target error is 0 (center of screen)

        # PID Output Limits (prevent excessive servo commands) - Adjust based on servo speed/range
        pid_out_limit_yaw = 50 # Max change in servo units per PID update step
        pid_out_limit_pitch = 50

        previous_time = time.time()
        st_frame_info = MV_FRAME_OUT() # Reusable structure for frame info
        memset(byref(st_frame_info), 0, sizeof(st_frame_info))


        # --- Main Processing Loop ---
        while not g_exit_flag:
            # --- Get Frame from Hikvision Camera ---
            frame_bgr = None
            ret = cam.MV_CC_GetImageBuffer(st_frame_info, 1000) # Timeout 1 sec

            if ret == 0:
                if st_frame_info.pBufAddr is not None and st_frame_info.stFrameInfo.nFrameLen > 0:
                    # --- Convert to BGR ---
                    st_convert_param = MV_CC_PIXEL_CONVERT_PARAM_EX()
                    memset(byref(st_convert_param), 0, sizeof(st_convert_param))
                    st_convert_param.nWidth = st_frame_info.stFrameInfo.nWidth
                    st_convert_param.nHeight = st_frame_info.stFrameInfo.nHeight
                    st_convert_param.pSrcData = st_frame_info.pBufAddr
                    st_convert_param.nSrcDataLen = st_frame_info.stFrameInfo.nFrameLen
                    st_convert_param.enSrcPixelType = st_frame_info.stFrameInfo.enPixelType
                    st_convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed # Target OpenCV format
                    st_convert_param.pDstBuffer = ctypes.cast(p_converted_data, ctypes.POINTER(ctypes.c_ubyte))
                    st_convert_param.nDstBufferSize = converted_data_size

                    convert_ret = cam.MV_CC_ConvertPixelTypeEx(st_convert_param)
                    if convert_ret == 0:
                        # Create numpy array from the *converted* buffer
                        img_data = np.frombuffer(p_converted_data, dtype=np.uint8, count=st_convert_param.nDstLen)
                        # Reshape (handle potential size mismatch)
                        expected_size = height * width * 3
                        if img_data.size == expected_size:
                            frame_bgr = img_data.reshape((height, width, 3))
                        else:
                            print(f"Warning: Converted buffer size mismatch. Got {img_data.size}, expected {expected_size}")
                            frame_bgr = None # Skip processing this frame
                    else:
                        print(f"Error: ConvertPixelTypeEx failed ({convert_ret:#x}) for pixel type {st_frame_info.stFrameInfo.enPixelType}")
                        frame_bgr = None

                    # --- Free SDK Buffer ---
                    free_ret = cam.MV_CC_FreeImageBuffer(st_frame_info)
                    # if free_ret != 0: print(f"Warning: FreeImageBuffer failed ({free_ret:#x})")

                else:
                    # print("Warning: GetImageBuffer success but pBufAddr is None or FrameLen is 0")
                    frame_bgr = None
                    # Need to free even if buffer is null? Check SDK docs. Usually not.

            elif ret == MV_E_TIMEOUT:
                # print("Timeout getting image buffer.")
                time.sleep(0.01) # Avoid busy-waiting
                continue # Skip rest of loop on timeout
            elif ret == MV_E_NODATA:
                 print("Warning: MV_E_NODATA received.")
                 time.sleep(0.1)
                 continue
            else:
                print(f"Error: GetImageBuffer failed! ret[0x{ret:x}]")
                g_exit_flag = True # Exit on critical error
                break

            # --- Process Frame (if obtained successfully) ---
            if frame_bgr is not None:
                # --- Armor Detection ---
                binary_img, color_mask = detector.preprocess_image(frame_bgr)
                lights = detector.find_lights(binary_img, frame_bgr, color_mask)
                armors = detector.match_lights(lights)

                # --- Visualization Image ---
                # Draw on the BGR frame or the binary image
                vis_img = frame_bgr.copy() # Draw on color image
                # Draw center crosshair
                cv2.line(vis_img, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)
                cv2.line(vis_img, (0, height // 2), (width, height // 2), (0, 255, 0), 1)

                # Draw detected lights
                for light in lights:
                     # Draw rotated rectangle
                     points = np.int0(cv2.boxPoints(light.box))
                     cv2.polylines(vis_img, [points], True, (0, 255, 0), 1) # Green for lights
                     # cv2.circle(vis_img, tuple(light.center), 3, (0, 255, 0), -1)

                # Draw detected armors
                target_locked = False
                if len(armors) > 0:
                    # Target the first detected armor (e.g., the leftmost one)
                    target_armor = armors[0]
                    target_locked = True

                    # Draw armor bounding box/line
                    l_center = tuple(np.int0(target_armor.left_light.center))
                    r_center = tuple(np.int0(target_armor.right_light.center))
                    armor_center = tuple(np.int0(target_armor.center))
                    cv2.line(vis_img, l_center, r_center, (0, 0, 255), 2) # Red line for armor
                    cv2.circle(vis_img, armor_center, 5, (0, 0, 255), -1) # Red circle for center
                    cv2.putText(vis_img, target_armor.type.value,
                                (armor_center[0] + 10, armor_center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    # --- PID Control ---
                    current_time = time.time()
                    dt = current_time - previous_time
                    if dt > 0: # Ensure dt is positive
                         previous_time = current_time

                         # Calculate error (pixels from center)
                         error_x = target_armor.center[0] - width // 2
                         error_y = target_armor.center[1] - height // 2

                         # Update PIDs and get output (change in angle)
                         # Note: Signs depend on servo orientation and coordinate system
                         # Assuming positive error_x means target is to the right -> need positive yaw command?
                         # Assuming positive error_y means target is below -> need positive pitch command? (Adjust signs!)
                         pid1_output = pid1.update(error_x, dt, -pid_out_limit_yaw, pid_out_limit_yaw)
                         pid2_output = pid2.update(-error_y, dt, -pid_out_limit_pitch, pid_out_limit_pitch) # Invert Y error for pitch

                         # Update target angle deviations
                         angle1_dev += pid1_output
                         angle2_dev += pid2_output

                         # Clamp angle deviations to allowed range (redundant if set_angle clamps)
                         # angle1_dev = np.clip(angle1_dev, ANGLE_RANGE_DEV[1][0], ANGLE_RANGE_DEV[1][1])
                         # angle2_dev = np.clip(angle2_dev, ANGLE_RANGE_DEV[2][0], ANGLE_RANGE_DEV[2][1])

                         # Send to servos
                         set_angle(1, angle1_dev)
                         set_angle(2, angle2_dev)

                         # Display PID info
                         cv2.putText(vis_img, f"Target: ({target_armor.center[0]},{target_armor.center[1]})", (10, height - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                         cv2.putText(vis_img, f"Error: ({error_x},{error_y})", (10, height - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                         cv2.putText(vis_img, f"PID Out: Yaw({pid1_output:.2f}) Pitch({pid2_output:.2f})", (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                         cv2.putText(vis_img, f"Servo Dev: Yaw({angle1_dev:.1f}) Pitch({angle2_dev:.1f})", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

                # Display target color
                cv2.putText(vis_img, f"Target Color: {detector.detect_color}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                if target_locked:
                    cv2.putText(vis_img, "LOCKED", (width - 100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # Display the frame
                cv2.imshow('Camera Feed', vis_img)
                # Optionally display binary image too
                cv2.imshow('Binary Image', binary_img)

            # --- Handle Keyboard Input ---
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Exit key pressed.")
                g_exit_flag = True
                break
            elif key == ord('c'):
                 detector.detect_color = 'BLUE' if detector.detect_color == 'RED' else 'RED'
                 print(f"Switched target color to: {detector.detect_color}")

    except Exception as e:
        print(f"An exception occurred in main: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback
        g_exit_flag = True

    finally:
        # --- Cleanup ---
        print("Starting cleanup...")
        g_exit_flag = True # Ensure flag is set for threads if they were used

        # Stop Camera Grabbing and Close
        if cam is not None:
            handle_check = ctypes.c_void_p(cam.handle)
            if handle_check:
                print("Stopping grabbing...")
                cam.MV_CC_StopGrabbing()
                print("Closing device...")
                cam.MV_CC_CloseDevice()
                print("Destroying handle...")
                cam.MV_CC_DestroyHandle()
            else:
                print("Camera handle already invalid or not created.")

        # Finalize SDK
        MvCamera.MV_CC_Finalize()
        print("MVS SDK Finalized.")

        # Close Serial Port
        if uart is not None and uart.is_open:
            print("Closing serial port...")
            uart.close()

        # Close OpenCV Windows
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")
        print("Program finished.")


if __name__ == "__main__":
    main()