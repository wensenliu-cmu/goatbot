# imu_interface.py
import serial
import time

# Initialize the serial port for the IMU (if separate from motors)
imu_ser = serial.Serial('COM4', 115200, timeout=1)  # adjust port and baudrate

def read_imu_data():
    """
    Reads data from the IMU sensor.
    
    Returns:
        dict: Example dictionary with keys 'roll', 'pitch', 'yaw', etc.
    """
    # This is a placeholder.
    # You might receive a line like "roll:1.2,pitch:-0.5,yaw:0.0\n"
    line = imu_ser.readline().decode('utf-8').strip()
    data = {}
    try:
        parts = line.split(',')
        for part in parts:
            key, value = part.split(':')
            data[key.strip()] = float(value.strip())
    except Exception as e:
        # If parsing fails, return dummy data or handle error.
        data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    return data

def close_imu():
    imu_ser.close()