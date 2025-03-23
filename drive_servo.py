import sys
import common.board_helper_functions as bhf

if __name__ == '__main__':
    helper = bhf.BoardHelper()
    print(f"Driving Hiwonder HTD-45H Bus Servo To Position")
    
    if len(sys.argv) == 4:
        time_to_pos = float(sys.argv[1])
        servo_id = int(sys.argv[2])
        angle = float(sys.argv[3])
        helper.set_servo_positions(time_to_pos=time_to_pos, servos=[servo_id], positions=[angle])

    else: print(f"Improper Arguments")