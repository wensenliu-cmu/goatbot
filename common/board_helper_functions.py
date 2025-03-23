import sys
import time
import signal
import threading
import common.ros_robot_controller_sdk as rrc

class BoardHelper:
    def __init__(self):
        self.board = rrc.Board()
        self.board.enable_reception()

        print(f"===== Initializing Expansion Board Helper =====")
        self.play_sound(1900, 0.1, 0.4, 3)
        time.sleep(2)
        self.play_sound(1000, 0.0, 0.0, 1)
        print(f"===== Expansion Board Initialized =====")

    def log_servo(self, servo_id: int) -> None:
        vin = self.board.bus_servo_read_vin(servo_id)
        vin = vin[0]/1000.0

        temp = self.board.bus_servo_read_temp(servo_id)
        temp = temp[0]

        position = self.board.bus_servo_read_position(servo_id)
        position = position[0]
        angle = self.pos_to_ang(pos=position)

        sID = f"ID: {servo_id}"
        sVin = f"Vin: {vin:.2f}V"
        sTemp = f"Temp: {temp:.2f}C"
        sPos = f"Pos: {position:.2f}"
        sAng = f"Angle: {angle:.2f}deg"
        log = f"{sID:<10}\t{sVin:<20}\t{sTemp:<20}\t{sPos:<20}\t{sAng:<20}\n"
        print("\n--- Logger: Servo Feedback ---")
        print(log)

    def identify_servo(self) -> None:
        try:
            servo_id = self.board.bus_servo_read_id()
            servo_id = servo_id[0]
            print(f"Servo ID: {servo_id}")
        except Exception as e:
            print(f"Exception: {e}")
            print(f"Failed to identify servo, do you have more than 1 servo attached?")

    def set_servo_id(self, old_id: int, new_id: int) -> bool:
        try: 
            self.board.bus_servo_set_id(servo_id_now=old_id, servo_id_new=new_id)
        except Exception as e: 
            print(f"Exception: {e}")
            print(f"Failed to set servo ID")
            return False

        return True
    
    def pos_to_ang(self, pos: int) -> float:
        return pos*0.24
    
    def ang_to_pos(self, ang: float) -> int:
        return int(ang/0.24)
    
    def play_sound(self, freq: int, on_time: float, off_time: float, num_repeats: int) -> None:
        self.board.set_buzzer(freq=freq, on_time=on_time, off_time=off_time, repeat=num_repeats)