# simple_head_shake.py
# -- coding: utf-8 --

import sys
import serial
import time

# --- Configuration ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # <<< IMPORTANT: Verify this port name!
SERVO_BAUDRATE = 115200
SLEEP_DURATION = 1.5  # Seconds between movements

# --- Servo Specific Configuration (Yaw Servo) ---
YAW_SERVO_ID = 1
YAW_MID_POS = 2047
YAW_MIN_DEV = -1400
YAW_MAX_DEV = 1400

# --- Attempt to import required classes ---
try:
    # Needed to initialize the communication channel
    from src.pans.uart_servo import UartServoManager
    # The library class we created
    from src.pans_control import GimbalServo
    print("Libraries imported successfully.")
except ImportError as e:
    print(f"Error importing required modules: {e}")
    print("Ensure 'gimbal_servo_lib.py' and 'src/pans/uart_servo.py' are accessible.")
    sys.exit("Exiting due to import error.")

# --- Global Variables ---
uart = None
uservo = None
yaw_servo = None

# --- Main Execution ---
try:
    # --- Initialize Serial and Servo Manager ---
    print(f"Attempting to open serial port {SERVO_PORT_NAME}...")
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0.1)

    # Initialize manager ONLY with the ID we intend to use for this simple script
    uservo = UartServoManager(uart, servo_id_list=[YAW_SERVO_ID])
    print(f"Serial port opened and UartServoManager initialized for ID: {YAW_SERVO_ID}.")

    # --- Initialize the specific servo using the library ---
    yaw_servo = GimbalServo(
        servo_id=YAW_SERVO_ID,
        mid_pos=YAW_MID_POS,
        min_dev=YAW_MIN_DEV,
        max_dev=YAW_MAX_DEV,
        uservo_manager=uservo  # Pass the manager instance
    )
    print("Yaw servo object created.")

    # --- Initial Centering ---
    print("Centering servo initially...")
    yaw_servo.center()
    time.sleep(1.0) # Give time for centering

    # --- Movement Loop ---
    print("\nStarting head shake loop (Press Ctrl+C to exit)...")
    while True:
        # Move to maximum deviation (e.g., right)
        print(f"Moving to half of max deviation ({YAW_MAX_DEV/2})...")
        yaw_servo.set_deviation(YAW_MAX_DEV/2)
        time.sleep(SLEEP_DURATION)

        # Move to minimum deviation (e.g., left)
        print(f"Moving to half of min deviation ({YAW_MIN_DEV/2})...")
        yaw_servo.set_deviation(YAW_MIN_DEV/2)
        time.sleep(SLEEP_DURATION)

except serial.SerialException as e:
    print(f"\nERROR: Could not open or communicate on serial port {SERVO_PORT_NAME}.")
    print(f"Serial Error: {e}")
    print("Please check connection, permissions, and port name.")
except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
    import traceback
    traceback.print_exc()

finally:
    # --- Cleanup ---
    print("\nInitiating cleanup...")
    if yaw_servo: # Check if servo object was successfully created
        print("Centering servo before exit...")
        try:
            yaw_servo.center()
            time.sleep(0.5) # Allow time for command
        except Exception as e:
            print(f"Error during final centering: {e}")

    if uart and uart.is_open:
        print("Closing serial port...")
        uart.close()
        print("Serial port closed.")
    else:
        print("Serial port was not open or not initialized.")

    print("Program finished.")
    sys.exit(0) # Explicitly exit