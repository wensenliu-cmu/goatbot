import sys
import board_helper_functions as bhf

if __name__ == '__main__':
    print(f"Identifying Attached Hiwonder HTD-45H Bus Servo")
    helper = bhf.BoardHelper()
    helper.identify_servo()