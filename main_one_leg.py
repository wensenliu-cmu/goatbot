# main_one_leg.py

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from ik import calc_angles
from trajectory import standing_foot, manual_traj, create_trajectory, sinusoidal_trajectory, sin_trajectory, semiElliptical_trajectory
import common.board_helper_functions as bhf
import convert_IK_to_motor
import csv
import datetime

def main():

    resetMode = False
    # Initialize Board Helper
    helper = bhf.BoardHelper()
    
    # Simulation parameters
    dt = 0.1                # time step in seconds
    total_time = 10.0        # total simulation time in seconds
    num_steps = int(total_time / dt)
    
    # Mode selection: "gait" for walking trajectory, "standing" for static stance.
    mode = "gait"
    
    # IK parameters (from our paper and design)
    params = {
        'foot_offset': 0.06,  # foot offset in meters
        'l1': 0.0936,         # upper link length in meters
        'l2': 0.18166,        # lower link length in meters
        'l3': 0              # additional lower link (set to 0 if both lower links are identical)
    }

    left_servos = {
        "FL" : 1, 
        "BR" : 4,
        "BL" : 5,
        "FR" : 8
    }

    right_servos = {
        "FL" : 2,
        "BR" : 3,
        "BL" : 6,
        "FR" : 7
    }
    
    # Gait trajectory parameters (with longer gait cycle for smoother simulation)
    T = 0.6                # gait cycle period in seconds
    step_length = 0.04     # step length in meters
    hd = 0.20              # desired foot height during support phase (m)
    fc = 0.07              # foot clearance during swing phase (m)
    phase = 0.0            # phase offset for this leg
    stance_y = -0.20
    clearance_upper = 0.07
    clearance_lower = 0.02
    
    # larger gait parameters
    x_amp = 0.04
    y_amp = 0.04
    x_off = 0.0
    y_off = -0.14
    f = 1/T
    
    # Precompute a high-resolution trajectory for one gait cycle (only for gait mode)

    if resetMode and mode == "gait":
        #helper.set_servo_positions(2, [1, 2], [135, 45])

        helper.set_servo_positions(1, list(left_servos.values()), [120] * len(left_servos))
        helper.set_servo_positions(1, list(right_servos.values()), [120] * len(right_servos))
        time.sleep(4)

        #helper.set_servo_positions(0.33, list(left_servos.values()), [135] * len(left_servos))
        #helper.set_servo_positions(0.33, list(right_servos.values()), [105] * len(right_servos))
        #time.sleep(4)
        
        #helper.set_servo_positions(0.67, list(left_servos.values()), [55] * len(left_servos))
        #helper.set_servo_positions(0.67, list(right_servos.values()), [185] * len(right_servos))
        time.sleep(4)

       # helper.set_servo_positions(1, list(left_servos.values()), [160] * len(left_servos))
       # helper.set_servo_positions(1, list(right_servos.values()), [80] * len(right_servos))
        time.sleep(4)
        
        resetMode = False

    if mode == "gait":
        num_samples = 4000  # high resolution sampling
        t_cycle = np.linspace(0, T, num_samples, endpoint=False)
        traj_cycle = np.array([semiElliptical_trajectory(t, T, phase,
                                 x_start=-0.01, step_length=step_length,
                                 stance_y=stance_y,
                                 clearance_upper=clearance_upper,
                                 clearance_lower=clearance_lower) for t in t_cycle])
    
    # Data logging for plotting
    time_history = []
    joint_history = []
    IMU_history = []
    desired_joint1_history = []
    desired_joint2_history = []

    measured_joint1_history = []
    measured_joint2_history = []
    
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        
        # --- 1. Determine Desired Foot Position ---
        if mode == "gait":
            tau1 = (t % T) / T   # normalized time [0,1)
            tau2 = (tau1 + 0.5) % 1
            index1 = int(tau1 * num_samples)
            index2 = int(tau2 * num_samples)
            desired_foot1 = traj_cycle[index1]
            desired_foot2 = traj_cycle[index2]
        else:
            desired_foot = standing_foot(hd)
        
        # --- 2. Compute Desired Joint Angles via IK ---
        # IK returns angles in radians.
        # 1 is trajectory for FR, BL - 2 is offest trajectory for FL, BR.
        LM_1_rad, RM_1_rad = calc_angles(desired_foot1[0], desired_foot1[1], params, "outward")
        LM_2_rad, RM_2_rad = calc_angles(desired_foot2[0], desired_foot2[1], params, "outward")

        # Convert IK angles to degrees.
        LM_1_deg, RM_1_deg = math.degrees(LM_1_rad), math.degrees(RM_1_rad)
        LM_2_deg, RM_2_deg = math.degrees(LM_2_rad), math.degrees(RM_2_rad)
        
        #print(f"left: {θ_deg_left_RS}\t right: {θ_deg_right_RS}")

        # Rotate planes to get joint angles.
        LM_1_motor = convert_IK_to_motor.convert_IK_to_motor(LM_1_deg, 'left')
        RM_1_motor = convert_IK_to_motor.convert_IK_to_motor(RM_1_deg, 'right')
        #print(f"left: {LS_left_motor}\t right: {LS_right_motor}")

        LM_2_motor = convert_IK_to_motor.convert_IK_to_motor(LM_2_deg, 'left')
        RM_2_motor = convert_IK_to_motor.convert_IK_to_motor(RM_2_deg, 'right')
        print(f"left: {LM_2_motor}\t right: {RM_2_motor}")

        # --- 7. Log Data ---
        time_history.append(t)
        joint_log = [LM_1_motor, RM_1_motor, LM_2_motor, RM_2_motor]
        joint_history.append(joint_log)
        
        IMU_reading = helper.read_IMU()
        if IMU_reading is not None:
            IMU_log = [IMU_reading[key] for key in IMU_reading]
            IMU_history.append(IMU_log)
        else:
            IMU_history.append([0, 0, 0, 0, 0, 0])

        #print(f"trajectory: {desired_foot1}\t {desired_foot2}")

        # Send data to servos
        #print(f"left: {theta2_motor1}\t right: {theta1_motor1}")
        #print(f"theta1_des: {240 - theta2_motor1}\t theta2_des: {240 - theta2_motor1}")
        
        
        # FRONT LEFT
        helper.set_servo_positions(dt, [1], [240 - RM_1_motor])
        helper.set_servo_positions(dt, [2], [240 - LM_1_motor])

        # FRONT RIGHT
        helper.set_servo_positions(dt, [8], [LM_2_motor])
        helper.set_servo_positions(dt, [7], [RM_2_motor])

        # BACK RIGHT
        helper.set_servo_positions(dt, [4], [LM_1_motor])
        helper.set_servo_positions(dt, [3], [RM_1_motor])
        
        # BACK LEFT
        helper.set_servo_positions(dt, [5], [240 - RM_2_motor])
        helper.set_servo_positions(dt, [6], [240 - LM_2_motor])
        
        #helper.set_servo_positions(3, [1, 2, 3, 4, 5, 6, 7, 8], [120, 120, 120, 120, 120, 120, 120, 120])

        
        # Maintain real-time simulation pace (optional)
        elapsed = time.time() - start_time
        if t > elapsed:
            time.sleep(t - elapsed)

    helper.set_servo_positions(1, list(left_servos.values()), [120] * len(left_servos))
    helper.set_servo_positions(1, list(right_servos.values()), [120] * len(right_servos))

    ground_type = "Big_Gravel"
    foot_type = "Goat"
    timestamp = int(datetime.datetime.now().timestamp())
    filename_joints = f"Data/{ground_type}_{foot_type}_joints_{timestamp}.csv"
    filename_IMU = f"Data/{ground_type}_{foot_type}_IMU_{timestamp}.csv"
    with open(filename_joints, 'w') as f:
        write = csv.writer(f)

        fields = ["LM1", "RM1", "LM2", "RM2"]
        write.writerow(fields)
        write.writerows(joint_history)

    with open(filename_IMU, 'w') as f:
        write = csv.writer(f)

        fields = ["Xacc", "Yacc", "Zacc", "Xvel", "Yvel", "Zvel"]
        write.writerow(fields)
        write.writerows(IMU_history)

if __name__ == '__main__':
    main()