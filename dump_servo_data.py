import sys
import board_helper_functions as bhf

if __name__ == '__main__':
    print(f"Dumping All Requested Hiwonder HTD-45H Bus Servo Data")
    helper = bhf.BoardHelper()
    helper.identify_servo()