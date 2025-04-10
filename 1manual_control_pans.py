# gimbal_control_gui_lib_user.py
# -- coding: utf-8 --

import sys
import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np # Still needed for the library potentially

# --- Attempt to import UartServoManager (needed for initialization here) ---
try:
    from src.pans.uart_servo import UartServoManager
    print("UartServoManager imported successfully in main script.")
except ImportError as e:
    print(f"Error importing UartServoManager in main script: {e}")
    print("Please ensure 'src/pans/uart_servo.py' exists and is accessible.")
    sys.exit("Exiting: UartServoManager is required for initialization.")

# --- Attempt to import the new GimbalServo library ---
try:
    # Assuming gimbal_servo_lib.py is in the same directory or Python path
    from src.pans_control import GimbalServo
    print("GimbalServo library imported successfully.")
except ImportError as e:
    print(f"Error importing GimbalServo library: {e}")
    print("Please ensure 'gimbal_servo_lib.py' is accessible.")
    sys.exit("Exiting: GimbalServo library is required.")


# --- Servo Configuration (Application specific) ---
SERVO_PORT_NAME = '/dev/ttyUSB0'  # <<< IMPORTANT: Verify this port name is correct!
SERVO_BAUDRATE = 115200
# Define which Servo IDs are used by this application
# We'll use index 0 for Yaw (ID 1) and index 1 for Pitch (ID 2)
SERVO_IDS_USED = [1, 2]
# Configuration mapping: Servo ID -> Parameters
# Using a dictionary for clarity if IDs are not sequential or sparse
SERVO_CONFIG = {
    1: { # Yaw Servo Configuration
        "mid_pos": 2047,
        "min_dev": -1400,
        "max_dev": 1400,
        "label": "Yaw"
    },
    2: { # Pitch Servo Configuration
        "mid_pos": 2047,
        "min_dev": -650,
        "max_dev": 1024,
        "label": "Pitch"
    }
    # Add more servos here if needed by changing SERVO_IDS_USED
    # and adding entries to SERVO_CONFIG
}


# --- Global Variables ---
uart = None
uservo = None
servo_manager_initialized = False
yaw_servo: GimbalServo | None = None   # Type hinting for clarity
pitch_servo: GimbalServo | None = None


# --- Initialize Serial and Servo Manager ---
try:
    print(f"Attempting to open serial port {SERVO_PORT_NAME}...")
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0.1)
    # Initialize UartServoManager with ALL servo IDs it might control on this port
    all_servo_ids_on_port = list(SERVO_CONFIG.keys()) # Or provide a known list if different
    uservo = UartServoManager(uart, servo_id_list=all_servo_ids_on_port)
    print(f"Serial port {SERVO_PORT_NAME} opened successfully.")
    print(f"UartServoManager initialized for IDs: {all_servo_ids_on_port}")
    servo_manager_initialized = True

    # --- Instantiate GimbalServo objects using the library ---
    # Check if ID 1 is used and configured
    if 1 in SERVO_IDS_USED and 1 in SERVO_CONFIG:
        cfg = SERVO_CONFIG[1]
        yaw_servo = GimbalServo(
            servo_id=1,
            mid_pos=cfg["mid_pos"],
            min_dev=cfg["min_dev"],
            max_dev=cfg["max_dev"],
            uservo_manager=uservo # Pass the shared manager instance
        )
    else:
         print("Warning: Yaw servo (ID 1) not configured or not in SERVO_IDS_USED.")

     # Check if ID 2 is used and configured
    if 2 in SERVO_IDS_USED and 2 in SERVO_CONFIG:
        cfg = SERVO_CONFIG[2]
        pitch_servo = GimbalServo(
            servo_id=2,
            mid_pos=cfg["mid_pos"],
            min_dev=cfg["min_dev"],
            max_dev=cfg["max_dev"],
            uservo_manager=uservo # Pass the shared manager instance
        )
    else:
         print("Warning: Pitch servo (ID 2) not configured or not in SERVO_IDS_USED.")

except serial.SerialException as e:
    print(f"Fatal Error opening serial port {SERVO_PORT_NAME}: {e}")
    print("Servo control will be disabled. Check connection and permissions.")
except NameError as e:
    print(f"Fatal Error: UartServoManager or GimbalServo class not found. Check imports. ({e})")
    print("Servo control will be disabled.")
except Exception as e:
    print(f"An unexpected error occurred during serial/servo setup: {e}")
    import traceback
    traceback.print_exc()
    print("Servo control will be disabled.")


# --- Tkinter GUI Setup ---
root = tk.Tk()
root.title("Gimbal Control (Library)")
root.geometry("300x280") # Slightly taller for status

# Status Label
status_text = tk.StringVar()
status_label = ttk.Label(root, textvariable=status_text, foreground="red")
status_label.pack(pady=(5,0))


# --- Slider Callback Functions ---
def yaw_slider_changed(value):
    """Called when the Yaw slider value changes."""
    if yaw_servo:
        yaw_servo.set_deviation(float(value))
        yaw_value_label.config(text=f"Yaw Dev: {float(value):.1f}")
    else:
         status_text.set("Yaw servo not available!")

def pitch_slider_changed(value):
    """Called when the Pitch slider value changes."""
    if pitch_servo:
        pitch_servo.set_deviation(float(value))
        pitch_value_label.config(text=f"Pitch Dev: {float(value):.1f}")
    else:
        status_text.set("Pitch servo not available!")


def center_gimbal():
    """Sets both servos to their middle position (0 deviation) via sliders."""
    if yaw_servo:
        yaw_slider.set(0)   # Let slider trigger the servo method via callback
    if pitch_servo:
        pitch_slider.set(0) # Let slider trigger the servo method via callback
    print("Centering gimbal (triggered via sliders).")
    status_text.set("Centering...")
    root.after(500, lambda: status_text.set("")) # Clear status after a delay


# --- Create GUI Widgets ---
# Style (optional)
style = ttk.Style()
try:
    # Use a theme if available (e.g., 'clam', 'alt', 'default', 'classic')
    style.theme_use('clam')
except tk.TclError:
    print("Note: 'clam' theme not available, using default.") # Fallback
style.configure("TScale", troughcolor='lightblue', sliderlength=25)
style.configure("TButton", padding=6, relief="flat", background="#e0e0e0")
style.configure("Red.TLabel", foreground="red") # For status

# Use servo labels from config if available
yaw_cfg = SERVO_CONFIG.get(1, {"label": "Yaw", "min_dev": -100, "max_dev": 100}) # Default fallback
pitch_cfg = SERVO_CONFIG.get(2, {"label": "Pitch", "min_dev": -100, "max_dev": 100})

# Yaw Slider (Horizontal)
yaw_widget_label = ttk.Label(root, text=f"{yaw_cfg['label']} Control (ID 1)")
yaw_widget_label.pack(pady=(10, 0))
yaw_slider = ttk.Scale(root, from_=yaw_cfg["min_dev"], to=yaw_cfg["max_dev"], orient=tk.HORIZONTAL,
                      length=250, command=yaw_slider_changed, style="TScale")
yaw_slider.pack(pady=(5, 0))
yaw_value_label = ttk.Label(root, text=f"Yaw Dev: 0.0")
yaw_value_label.pack()

# Pitch Slider (Vertical)
pitch_widget_label = ttk.Label(root, text=f"{pitch_cfg['label']} Control (ID 2)")
pitch_widget_label.pack(pady=(10, 0))
# Reversed from/to for intuitive up/down GUI control
pitch_slider = ttk.Scale(root, from_=pitch_cfg["max_dev"], to=pitch_cfg["min_dev"], orient=tk.VERTICAL,
                       length=100, command=pitch_slider_changed, style="TScale")
pitch_slider.pack(pady=(5, 0))
pitch_value_label = ttk.Label(root, text=f"Pitch Dev: 0.0")
pitch_value_label.pack()

# Center Button
center_button = ttk.Button(root, text="Center Gimbal", command=center_gimbal, style="TButton")
center_button.pack(pady=15)


# --- Set Initial State ---
if servo_manager_initialized and (yaw_servo or pitch_servo):
    status_text.set("Servos Initialized")
    status_label.config(foreground="green")
    if yaw_servo:
        yaw_slider.set(0)
        # yaw_servo.center() # Explicitly center if slider callback doesn't fire initially
    else:
        yaw_slider.config(state=tk.DISABLED)
        yaw_widget_label.config(text=f"{yaw_cfg['label']} Control (DISABLED)")

    if pitch_servo:
        pitch_slider.set(0)
        # pitch_servo.center() # Explicitly center if slider callback doesn't fire initially
    else:
        pitch_slider.config(state=tk.DISABLED)
        pitch_widget_label.config(text=f"{pitch_cfg['label']} Control (DISABLED)")

    # Center servos explicitly on start after sliders are set
    root.after(100, center_gimbal) # Small delay to ensure GUI updates

else:
    # Disable all widgets if servo initialization failed or no servos created
    status_text.set("SERVO ERROR - Check Console")
    status_label.config(foreground="red")
    yaw_slider.config(state=tk.DISABLED)
    pitch_slider.config(state=tk.DISABLED)
    center_button.config(state=tk.DISABLED)
    yaw_widget_label.config(text=f"{yaw_cfg['label']} Control (ERROR)")
    pitch_widget_label.config(text=f"{pitch_cfg['label']} Control (ERROR)")
    root.title("Gimbal Control (SERVO ERROR)")


# --- Cleanup Function ---
def on_closing():
    """Handles window closing event."""
    print("Closing application...")
    if servo_manager_initialized and uart is not None and uart.is_open:
        print("Centering servos before closing...")
        try:
            # Use the servo objects to center
            if yaw_servo:
                yaw_servo.center()
                time.sleep(0.1) # Small delay for command to be sent/processed
            if pitch_servo:
                pitch_servo.center()
                time.sleep(0.1)
        except Exception as e:
            print(f"Error centering servos on close: {e}")
        finally:
            print("Closing serial port...")
            uart.close()
            print("Serial port closed.")
    elif uart and uart.is_open:
         print("Closing serial port (manager might not have initialized)...")
         uart.close()
         print("Serial port closed.")
    else:
        print("Serial port was not open or not initialized.")
    root.destroy() # Close the Tkinter window

# Bind the closing function to the window close button
root.protocol("WM_DELETE_WINDOW", on_closing)

# --- Start Tkinter Main Loop ---
print("Starting GUI. Close the window or press Ctrl+C in terminal to exit.")
try:
    root.mainloop()
except KeyboardInterrupt:
    print("\nKeyboard interrupt detected.")
    on_closing()