# main_quadruped_stand.py

import time
import math
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ik import inverse_kinematics
from trajectory import standing_foot
import common.board_helper_functions as bhf
import convert_IK_to_motor


def main():

    # Initialize Board Helper
    helper = bhf.BoardHelper()

    dt = 0.01                # time step in seconds
    total_time = 10         # total simulation time in seconds
    num_steps = int(total_time / dt)
    
    # Mode is fixed: "standing" mode
    mode = "standing"
    
    # IK parameters (as before)
    params = {
        'foot_offset': 0.06,  # foot offset in meters
        'l1': 0.0936,         # upper link length in meters
        'l2': 0.18166,        # lower link length in meters
        'l3': 0              # additional lower link (set to 0 if identical)
    }
    
    hd = 0.26  # foot height for standing (m)
    # For standing, the foot position remains constant (e.g., directly under the hip).
    desired_foot_standing = standing_foot(hd)  # returns (x, y)
    
    # Define the four legs
    legs = ["FL", "FR", "BL", "BR"]
    
    # Initialize current joint angles for each leg (simulate both joint1 and joint2, in degrees)
    current_angles = {leg: {"joint1": 0.0, "joint2": 0.0} for leg in legs}
    

    # Data logging for one leg (e.g., FL joint1)
    time_history = []
    desired_FL_joint1 = []
    measured_FL_joint1 = []
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        time_history.append(t)
        
        # For standing, the desired foot position remains constant for all legs.
        #for leg in legs:
        desired_foot = desired_foot_standing
        theta1_stand, theta2_stand = inverse_kinematics(desired_foot_standing[0], desired_foot_standing[1], params)
        
        # Rotate planes to get joint angles b/w 0-240
        # If servo1 is on the left and servo2 is on the right:
        theta1_motor = convert_IK_to_motor.convert_IK_to_motor(theta1_stand, 'left')
        theta2_motor = convert_IK_to_motor.convert_IK_to_motor(theta2_stand, 'right')

        # Send data to servos
        print(f"theta1_des: {theta1_motor}\t theta2_des: {theta2_motor}")
        helper.set_servo_positions(dt, [1, 2], [theta1_motor, theta2_motor])
        helper.set_servo_positions(dt, [4, 3], [theta1_motor, theta2_motor])
        helper.set_servo_positions(dt, [5, 6], [theta1_motor, theta2_motor])
        helper.set_servo_positions(dt, [7, 8], [theta1_motor, theta2_motor])    

        
        elapsed = time.time() - start_time
        if t > elapsed:
            time.sleep(t - elapsed)
    
    # Plot results for one leg (FL joint1)
    plt.figure()
    plt.plot(time_history, desired_FL_joint1, label="Desired FL Joint1")
    plt.plot(time_history, measured_FL_joint1, label="Measured FL Joint1")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint1 Angle (deg)")
    plt.title("Full Quadruped Standing: FL Joint1 Tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()