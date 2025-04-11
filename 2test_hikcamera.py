from src.hikvision_capture import HikvisionCamera, HikvisionCameraError
import sys
import cv2
import time

try:
    HikvisionCamera.initialize_sdk()

    with HikvisionCamera(device_index=0, verbose=False) as cam: # Use verbose=True for debug info
        if not cam.is_open():
            print("Error: Could not open camera.")
            sys.exit()

        width, height = cam.get_resolution()
        print(f"Camera Resolution: {width}x{height}")

        if not cam.start():
             print("Error: Could not start grabbing.")
             sys.exit()

        # --- Setup your detector, PID, servos here ---
        # detector = Detector(...)
        # pid1 = PID(...)
        # ...

        # --- Main Loop ---
        while True:
            ret, frame_bgr = cam.read() # Get BGR frame

            if not ret:
                # Handle frame grab failure (e.g., timeout)
                time.sleep(0.01) # Avoid busy-waiting
                continue

            # --- Process frame_bgr with your detector ---
            # binary_img, color_mask = detector.preprocess_image(frame_bgr)
            # lights = detector.find_lights(binary_img, frame_bgr, color_mask)
            # armors = detector.match_lights(lights)
            # ... PID control logic using armor position ...
            # ... Visualization ...

            # Display using OpenCV (example)
            # vis_img = frame_bgr.copy() # Or draw on your processed image
            # ... draw detections, PID info etc. on vis_img ...
            cv2.imshow("Processed Feed", frame_bgr) # Show the raw feed or processed image
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            # elif key == ord('c'):
            #     detector.detect_color = ...

    # cam.release() is called automatically by 'with' statement exit

except HikvisionCameraError as e:
    print(f"Camera Error: {e}")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    HikvisionCamera.finalize_sdk()
    cv2.destroyAllWindows()
    # Close serial port if open
    # if uart is not None and uart.is_open: uart.close()
    print("Application finished.")