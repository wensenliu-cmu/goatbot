# main_quadruped_stand.py

import time
import math
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ik import inverse_kinematics
from trajectory import standing_foot

def simulate_plant(current_angle, control_output, dt):
    """
    Simulate joint dynamics with a simple integrator model.
    """
    return current_angle + control_output * dt

def main():
    dt = 0.01                # time step in seconds
    total_time = 5.0         # total simulation time in seconds
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
    
    # Create PID controllers for each joint on each leg
    Kp, Ki, Kd = 5.5, 1.4, 0.34
    pid_controllers = {}
    for leg in legs:
        pid_controllers[leg] = {
            "joint1": PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0)),
            "joint2": PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0))
        }
    
    # Data logging for one leg (e.g., FL joint1)
    time_history = []
    desired_FL_joint1 = []
    measured_FL_joint1 = []
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        time_history.append(t)
        
        # For standing, the desired foot position remains constant for all legs.
        for leg in legs:
            desired_foot = desired_foot_standing
            theta1_des, theta2_des = inverse_kinematics(desired_foot[0], desired_foot[1], params)
            pid_controllers[leg]["joint1"].setpoint = theta1_des
            pid_controllers[leg]["joint2"].setpoint = theta2_des
            
            measured_joint1 = current_angles[leg]["joint1"]
            measured_joint2 = current_angles[leg]["joint2"]
            
            control_output1 = pid_controllers[leg]["joint1"].update(measured_joint1, dt)
            control_output2 = pid_controllers[leg]["joint2"].update(measured_joint2, dt)
            
            current_angles[leg]["joint1"] = simulate_plant(measured_joint1, control_output1, dt)
            current_angles[leg]["joint2"] = simulate_plant(measured_joint2, control_output2, dt)
            
            # Log data for FL leg as an example
            if leg == "FL":
                desired_FL_joint1.append(theta1_des)
                measured_FL_joint1.append(measured_joint1)
        
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