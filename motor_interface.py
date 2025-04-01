# motor_interface.py
import serial
import time

# Example: Initialize the serial port (adjust parameters for your hardware)
# Replace 'COM3' with your port or '/dev/ttyUSB0' on Linux
ser = serial.Serial('COM3', 115200, timeout=1)

def send_motor_command(joint_id, command):
    """
    Sends a command to the motor controlling the given joint.
    
    Args:
        joint_id (int or str): Identifier for the joint.
        command (float): The control signal (e.g., torque, PWM value, etc.).
    """
    # Format your command string as needed by your motor controller.
    # For example: "J1: 5.23\n" might mean Joint 1, command 5.23.
    cmd_str = f"{joint_id}:{command:.2f}\n"
    
    # Send the command via serial
    ser.write(cmd_str.encode('utf-8'))
    # Optionally, read response:
    # response = ser.readline().decode('utf-8').strip()
    # print(f"Response from motor {joint_id}: {response}")

# Optionally, add a cleanup function
def close_connection():
    ser.close()