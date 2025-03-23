import sys
import board_helper_functions as bhf

if __name__ == '__main__':
    print(f"Setting IDs for Hiwonder HTD-45H Bus Servo")
    helper = bhf.BoardHelper()

    if len(sys.argv) == 3:
        old_id = int(sys.argv[1])
        new_id = int(sys.argv[2])
        print(f"Attempting to set/reset servo ID {old_id} to ID {new_id}")
        set_id = helper.set_servo_id(old_id=old_id, new_id=new_id)

        if set_id:
            print(f"Successfully set servo ID")
            helper.log_servo(servo_id=new_id)

    else: print(f"Improper Arguments")