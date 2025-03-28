"""
Helper class for common functions when using the Raspberry Pi Expansion Board Model B from Hiwonder. 
Built on top of the ros_robot_controller_sdk provided by Hiwonder

Board Resources: https://drive.google.com/drive/folders/10olVCee9dYqQ2Swm085caoXdfCOFNc2F
Board Site: https://www.hiwonder.com/products/expansion-board-for-raspberry-pi-5?variant=40939498799191
"""

import sys
import math
import time
import signal
import threading
import common.ros_robot_controller_sdk as rrc

class BoardHelper:

    def __init__(self):
        """
        Intializes a Hiwonder Board Class and enables reception of feedback from the servo and IMU data

        ### Parameters
        No user provided parameters.

        ### Returns
        *BoardHelper : BoardHelper* -- A board helper class
        """

        self.board = rrc.Board()
        self.board.enable_reception()
        self.doInit = False

        if self.doInit:
            print(f"===== Initializing Expansion Board Helper =====")
            self.play_sound(1900, 0.1, 0.4, 3)
            time.sleep(2)
            self.play_sound(1000, 0.0, 0.0, 1)
            print(f"===== Expansion Board Initialized =====")

    def log_servo(self, servo_id: int) -> tuple:
        """
        Logs feedback data from the servo. 

        ### Parameters
        *servo_id : int* -- the id of the servo that the user is requesting data from

        ### Returns
        A tuple containing the following values (vin, temp, position, angle) \n
        *vin : float* -- the voltage being provided to the servo (in Volts) \n
        *temp : float* -- the temperature of the servo (in Celcius) \n
        *position : float* -- the angular position of the servo in servo coordinates (0-1000) \n
        *angle : float* -- the angular position of the servo in degrees (0-240)
        """

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
        print("--- Logger: Servo Feedback ---")
        print(log)

        return vin, temp, position, angle

    def identify_servo(self) -> None:
        """
        Identifies the servo currently plugged into the board. Only works with a single attached servo. 

        ### Parameters
        No user provided parameters.

        ### Returns
        Prints out the ID of the attached servo.
        """

        try:
            servo_id = self.board.bus_servo_read_id()
            servo_id = servo_id[0]
            print(f"Servo ID: {servo_id}")
        except Exception as e:
            print(f"Exception: {e}")
            print(f"Failed to identify servo, do you have more than 1 servo attached?")

    def set_servo_id(self, old_id: int, new_id: int) -> bool:
        """
        Attempts to change the ID of the servo tied to old_id to new_id. 

        ### Parameters
        *old_id : int* -- the id of the servo that the user is attempting to reassign
        *new_id : int* -- the id to reassign the servo to

        ### Returns
        A boolean indicating whether or not the assignment was successful \n
        """

        try: 
            self.board.bus_servo_set_id(servo_id_now=old_id, servo_id_new=new_id)
        except Exception as e: 
            print(f"Exception: {e}")
            print(f"Failed to set servo ID")
            return False

        return True
    
    def pos_to_ang(self, pos: int) -> float:
        """
        Maps servo position in servo coordinates (1-1000) to angular coordinates in degrees (0-240). 

        ### Parameters
        *pos : int* -- the servo position in servo coordinates (1-1000)

        ### Returns
        The angular position of the servo in degrees (float 0-240) \n
        """

        return pos*0.24
    
    def ang_to_pos(self, ang: float) -> int:
        """
        Maps servo position in angular coordinates in degrees (0-240) to servo coordinates (0-1000). 

        ### Parameters
        *ang : float* -- the servo position in angular coordinates in degrees (0-240)

        ### Returns
        The angular position of the servo in servo coordinates (int 0-1000) \n
        """

        return math.ceil(ang/0.24)
    
    def play_sound(self, freq: int, on_time: float, off_time: float, num_repeats: int) -> None:
        """
        Plays a sound on the servo's piezoelectric buzzer. 

        ### Parameters
        *servo_id : int* -- the id of the servo that the user is requesting data from

        ### Returns
        A tuple containing the following values (vin, temp, position, angle) \n
        *vin : float* -- the voltage being provided to the servo (in Volts) \n
        *temp : float* -- the temperature of the servo (in Celcius) \n
        *position : float* -- the angular position of the servo in servo coordinates (0-1000) \n
        *angle : float* -- the angular position of the servo in degrees (0-240)
        """

        self.board.set_buzzer(freq=freq, on_time=on_time, off_time=off_time, repeat=num_repeats)

    def set_servo_positions(self, time_to_pos: float, servos: list, positions: list) -> bool:
        positions = [self.ang_to_pos(position) for position in positions]
        command = [[servo, position] for servo, position in zip(servos, positions)]
        self.board.bus_servo_set_position(time_to_pos, command)