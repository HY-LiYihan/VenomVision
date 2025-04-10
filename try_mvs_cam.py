# -- coding: utf-8 --

import sys
import ctypes
import numpy as np
import cv2
import threading
import time

# Assuming MvCameraControl_class is in the specified path
# Adjust the path if your SDK installation differs
try:
    sys.path.append("/opt/MVS/Samples/64/Python/MvImport")
    from MvCameraControl_class import *
except ImportError:
    print("Error: MvCameraControl_class not found.")
    print("Please check the sys.path.append() line and ensure the SDK is installed correctly.")
    sys.exit()

# Global variable to signal thread termination
g_exit_flag = False

def display_thread(cam, st_device_list):
    """Thread function to continuously grab and display images."""
    global g_exit_flag

    try:
        # --- Setup ---
        # Get camera width and height *before* starting grab
        st_width = MVCC_INTVALUE_EX()
        st_height = MVCC_INTVALUE_EX()
        memset(byref(st_width), 0, sizeof(st_width))
        memset(byref(st_height), 0, sizeof(st_height))

        ret = cam.MV_CC_GetIntValueEx("Width", st_width)
        if ret != 0:
            print(f"Error: Get Width fail! ret[0x{ret:x}]")
            g_exit_flag = True
            return
        ret = cam.MV_CC_GetIntValueEx("Height", st_height)
        if ret != 0:
            print(f"Error: Get Height fail! ret[0x{ret:x}]")
            g_exit_flag = True
            return

        width = st_width.nCurValue
        height = st_height.nCurValue
        print(f"Camera resolution: {width} x {height}")

        # --- Buffer for Conversion ---
        # We will convert image data to BGR8 for OpenCV display
        # Allocate buffer for the converted image data
        # BGR8 format means 3 bytes per pixel
        converted_data_size = width * height * 3
        p_converted_data = (ctypes.c_ubyte * converted_data_size)()

        # --- Start Grabbing ---
        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            print(f"Error: Start grabbing fail! ret[0x{ret:x}]")
            g_exit_flag = True
            return
        print("Camera stream started. Press 'q' in the OpenCV window to exit.")

        # --- Frame Acquisition and Display Loop ---
        st_frame_info = MV_FRAME_OUT()
        memset(byref(st_frame_info), 0, sizeof(st_frame_info))
        img_buff = None

        while not g_exit_flag:
            # Get one frame from the camera buffer
            # Use a timeout (e.g., 1000ms)
            ret = cam.MV_CC_GetImageBuffer(st_frame_info, 1000)
            if ret == 0:
                # --- Process Image ---
                # print(f"GetOneFrame: W:{st_frame_info.stFrameInfo.nWidth}, H:{st_frame_info.stFrameInfo.nHeight}, FrameNum:{st_frame_info.stFrameInfo.nFrameNum}")

                # Check if the buffer address is valid
                if st_frame_info.pBufAddr is None:
                    print("Warning: Got frame info but pBufAddr is None.")
                    continue # Skip this frame

                # --- Convert Pixel Format to BGR8 ---
                st_convert_param = MV_CC_PIXEL_CONVERT_PARAM_EX()
                memset(byref(st_convert_param), 0, sizeof(st_convert_param))
                st_convert_param.nWidth = st_frame_info.stFrameInfo.nWidth
                st_convert_param.nHeight = st_frame_info.stFrameInfo.nHeight
                st_convert_param.pSrcData = st_frame_info.pBufAddr
                st_convert_param.nSrcDataLen = st_frame_info.stFrameInfo.nFrameLen
                st_convert_param.enSrcPixelType = st_frame_info.stFrameInfo.enPixelType
                st_convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed # Target format for OpenCV
                st_convert_param.pDstBuffer = ctypes.cast(p_converted_data, ctypes.POINTER(ctypes.c_ubyte))
                st_convert_param.nDstBufferSize = converted_data_size

                convert_ret = cam.MV_CC_ConvertPixelTypeEx(st_convert_param)
                if convert_ret != 0:
                    print(f"Error: Convert pixel type fail! ret[0x{convert_ret:x}]")
                    # Don't free buffer here if conversion failed, might still need it
                    # Might want to break or continue depending on desired robustness
                    continue # Skip frame display if conversion fails

                # --- Create OpenCV Image ---
                # Create a numpy array from the *converted* data buffer
                img_bgr = np.frombuffer(p_converted_data, dtype=np.uint8, count=st_convert_param.nDstLen)
                # Reshape the 1D array into a 2D image (Height x Width x Channels)
                # Ensure dimensions match the converted output
                if img_bgr.size == width * height * 3:
                     img_buff = img_bgr.reshape((height, width, 3))
                else:
                     print(f"Warning: Converted data size mismatch. Expected {width*height*3}, got {img_bgr.size}. Skipping frame.")
                     # Free the original buffer before continuing
                     if st_frame_info.pBufAddr is not None:
                         cam.MV_CC_FreeImageBuffer(st_frame_info)
                     continue

                # --- Display Image ---
                cv2.imshow("Camera Feed", img_buff)

                # --- Release SDK Buffer ---
                # Crucial step: Free the buffer allocated by the SDK
                if st_frame_info.pBufAddr is not None:
                    free_ret = cam.MV_CC_FreeImageBuffer(st_frame_info)
                    if free_ret != 0:
                        # This is unusual, might indicate a deeper problem
                        print(f"Warning: Failed to free image buffer! ret[0x{free_ret:x}]")

                # --- Check for Exit Key ---
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Exit key ('q') pressed.")
                    g_exit_flag = True
                    break

            elif ret == MV_E_TIMEOUT:
                # print("Timeout getting image buffer.")
                # If timeout, just continue trying
                 time.sleep(0.01) # Small sleep to prevent busy-waiting
                 continue
            elif ret == MV_E_NODATA:
                 print("Warning: MV_E_NODATA received.")
                 time.sleep(0.1) # Wait a bit longer if no data
                 continue
            else:
                print(f"Error: Get Image Buffer fail! ret[0x{ret:x}]")
                g_exit_flag = True
                break # Exit loop on critical error

    except Exception as e:
        print(f"Exception in display thread: {e}")
        g_exit_flag = True # Signal main thread to exit
    finally:
        # --- Cleanup within the thread (optional, main does most) ---
        print("Display thread finishing...")
        cv2.destroyAllWindows() # Close OpenCV window if thread stops


def main():
    """Main function to initialize SDK, find camera, and start display thread."""
    global g_exit_flag
    cam = None # Initialize cam to None for cleanup check

    try:
        # --- Initialize SDK ---
        MvCamera.MV_CC_Initialize()
        print("SDK Initialized.")

        # --- Enumerate Devices ---
        device_list = MV_CC_DEVICE_INFO_LIST()
        tlayer_type = (MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE
                       | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE)

        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if ret != 0:
            print(f"Error: Enum devices fail! ret[0x{ret:x}]")
            return # No need for sys.exit() here, just return

        if device_list.nDeviceNum == 0:
            print("Error: No devices found!")
            return

        print(f"Found {device_list.nDeviceNum} devices!")

        # --- Print Device Information (Simplified) ---
        for i in range(device_list.nDeviceNum):
            mvcc_dev_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            print(f"[{i}] Device Type: {mvcc_dev_info.nTLayerType}")
            # You can add more detailed info printing here if needed (like in your original code)

        # --- Select Device ---
        if device_list.nDeviceNum == 1:
            n_connection_num = 0
            print("Automatically selecting the only device found (index 0).")
        else:
            while True:
                try:
                    n_connection_num_str = input(f"Please input the index of the device to connect (0-{device_list.nDeviceNum - 1}): ")
                    n_connection_num = int(n_connection_num_str)
                    if 0 <= n_connection_num < device_list.nDeviceNum:
                        break
                    else:
                        print("Invalid input. Please enter a valid index.")
                except ValueError:
                    print("Invalid input. Please enter a number.")

        # --- Create Camera Instance ---
        cam = MvCamera()
        st_device_list = cast(device_list.pDeviceInfo[n_connection_num], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(st_device_list)
        if ret != 0:
            raise Exception(f"Create handle fail! ret[0x{ret:x}]")
        print(f"Handle created for device {n_connection_num}.")

        # --- Open Device ---
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception(f"Open device fail! ret[0x{ret:x}]")
        print("Device opened successfully.")

        # --- Optimize Packet Size (GigE Only) ---
        if st_device_list.nTLayerType == MV_GIGE_DEVICE or st_device_list.nTLayerType == MV_GENTL_GIGE_DEVICE:
            n_packet_size = cam.MV_CC_GetOptimalPacketSize()
            if int(n_packet_size) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", n_packet_size)
                if ret != 0:
                    print(f"Warning: Set Packet Size fail! ret[0x{ret:x}]")
                else:
                    print(f"Optimal packet size set to {n_packet_size}.")
            else:
                print(f"Warning: Get Optimal Packet Size fail! ret[0x{n_packet_size:x}]")

        # --- Set Trigger Mode Off (for continuous streaming) ---
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            # Non-critical, maybe camera was already in continuous mode
             print(f"Warning: Set trigger mode off fail! ret[0x{ret:x}]")
        else:
            print("Trigger mode set to Off (Continuous).")

        # --- Start Display Thread ---
        # Pass the camera instance and device info to the thread
        h_display_thread = threading.Thread(target=display_thread, args=(cam, st_device_list))
        h_display_thread.start()

        # --- Wait for Display Thread to Finish ---
        # The main thread will wait here until the display thread exits
        # (either by pressing 'q' or due to an error)
        while h_display_thread.is_alive() and not g_exit_flag:
             time.sleep(0.1) # Prevent busy-waiting

        # Ensure flag is set if thread exited cleanly via 'q'
        g_exit_flag = True
        h_display_thread.join() # Wait for thread cleanup if needed

    except Exception as e:
        print(f"An exception occurred in main: {e}")
        g_exit_flag = True # Signal exit in case thread is still running

    finally:
        # --- Cleanup ---
        print("Starting cleanup...")
        if cam is not None:
            # Check if handle exists before trying to destroy
             handle_check = ctypes.c_void_p(cam.handle)
             if handle_check:
                 print("Stopping grabbing...")
                 stop_ret = cam.MV_CC_StopGrabbing()
                 if stop_ret != 0:
                     print(f"Warning: Stop grabbing failed! ret[0x{stop_ret:x}]")

                 print("Closing device...")
                 close_ret = cam.MV_CC_CloseDevice()
                 if close_ret != 0:
                      print(f"Warning: Close device failed! ret[0x{close_ret:x}]")

                 print("Destroying handle...")
                 destroy_ret = cam.MV_CC_DestroyHandle()
                 if destroy_ret != 0:
                      print(f"Warning: Destroy handle failed! ret[0x{destroy_ret:x}]")
             else:
                print("Camera handle already null or not created.")
        else:
             print("Camera object not created.")

        # --- Finalize SDK ---
        MvCamera.MV_CC_Finalize()
        print("SDK Finalized.")
        print("Program finished.")


if __name__ == "__main__":
    main()