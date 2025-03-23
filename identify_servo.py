import sys
import common.board_helper_functions as bhf

if __name__ == '__main__':
    helper = bhf.BoardHelper()
    print(f"Identifying Attached Hiwonder HTD-45H Bus Servo")
    helper.identify_servo()