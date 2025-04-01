# main_one_leg.py

import time
import math
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ik import inverse_kinematics
from trajectory import foot_trajectory, standing_foot

def simulate_plant(current_angle, control_output, dt):
    """
    Simulate joint dynamics with a simple integrator model.
    """
    return current_angle + control_output * dt

def main():
    # Simulation parameters
    dt = 0.01                # time step in seconds
    total_time = 5.0         # total simulation time in seconds
    num_steps = int(total_time / dt)
    
    # Mode selection: "gait" for walking trajectory, "standing" for static stance.
    # For this file we use "gait" mode.
    mode = "gait"
    
    # IK parameters (from our paper and design)
    params = {
        'foot_offset': 0.06,  # foot offset in meters
        'l1': 0.0936,         # upper link length in meters
        'l2': 0.18166,        # lower link length in meters
        'l3': 0              # additional lower link (set to 0 if both lower links are identical)
    }
    
    # Gait trajectory parameters
    T = 0.4                # gait cycle period in seconds
    step_length = 0.12     # step length in meters
    hd = 0.26              # desired foot height during support phase (m)
    fc = 0.06              # foot clearance during swing phase (m)
    phase = 0.0            # phase offset for this leg (for gait mode)
    
    # Initialize simulated joint angles (in degrees) for the two joints
    current_angle1 = 0.0
    current_angle2 = 0.0
    
    # Create PID controllers for joint1 and joint2
    Kp, Ki, Kd = 5.5, 1.4, 0.34
    pid1 = PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0))
    pid2 = PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0))
    
    # Data logging for plotting
    time_history = []
    desired_joint1_history = []
    desired_joint2_history = []
    measured_joint1_history = []
    measured_joint2_history = []
    command1_history = []
    command2_history = []
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        
        # --- 1. Get desired foot position ---
        if mode == "gait":
            # Generate a dynamic semi-elliptical trajectory
            desired_foot = foot_trajectory(t, T, phase, step_length, hd, fc)
        else:  # mode "standing"
            desired_foot = standing_foot(hd)
        
        # --- 2. Compute desired joint angles via IK (returns angles in degrees) ---
        theta1_des, theta2_des = inverse_kinematics(desired_foot[0], desired_foot[1], params)
        
        # --- 3. Update PID setpoints ---
        pid1.setpoint = theta1_des
        pid2.setpoint = theta2_des
        
        # --- 4. Get simulated measured joint angles ---
        measured_angle1 = current_angle1
        measured_angle2 = current_angle2
        
        # --- 5. Compute PID control outputs ---
        control_output1 = pid1.update(measured_angle1, dt)
        control_output2 = pid2.update(measured_angle2, dt)
        
        # --- 6. Update the simulated plant dynamics ---
        current_angle1 = simulate_plant(measured_angle1, control_output1, dt)
        current_angle2 = simulate_plant(measured_angle2, control_output2, dt)
        
        # --- 7. Log Data ---
        time_history.append(t)
        desired_joint1_history.append(theta1_des)
        desired_joint2_history.append(theta2_des)
        measured_joint1_history.append(measured_angle1)
        measured_joint2_history.append(measured_angle2)
        command1_history.append(control_output1)
        command2_history.append(control_output2)
        
        # Maintain real-time simulation pace (optional)
        elapsed = time.time() - start_time
        if t > elapsed:
            time.sleep(t - elapsed)
    
    # --- 8. Plot the results ---
    plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_history, desired_joint1_history, label="Desired Joint1")
    plt.plot(time_history, measured_joint1_history, label="Measured Joint1")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint1 Angle (deg)")
    plt.title("One Leg: Joint1 Tracking (Gait Mode)")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(time_history, desired_joint2_history, label="Desired Joint2")
    plt.plot(time_history, measured_joint2_history, label="Measured Joint2")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint2 Angle (deg)")
    plt.title("One Leg: Joint2 Tracking (Gait Mode)")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    plt.figure(figsize=(10, 4))
    plt.plot(time_history, command1_history, label="PID Output Joint1")
    plt.plot(time_history, command2_history, label="PID Output Joint2")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Signal")
    plt.title("PID Controller Outputs (One Leg)")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()