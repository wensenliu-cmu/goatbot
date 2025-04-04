import sys
import common.board_helper_functions as bhf

if __name__ == '__main__':
    helper = bhf.BoardHelper()
    print(f"Dumping All Requested Hiwonder HTD-45H Bus Servo Data")

    if len(sys.argv) > 1:
        servos = sys.argv[1:]
        for servo in servos:
            servo_id = int(servo)
            try:
                helper.log_servo(servo_id=servo_id)
            except Exception as e:
                print(f"Servo {servo_id}:\tERROR: {e}")

    else: print(f"ERROR: No servo ID given")