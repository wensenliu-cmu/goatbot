# main.py

import time
import math
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ik import inverse_kinematics
from trajectory import foot_trajectory

# Motor interface
import common.board_helper_functions as bhf

# Dummy plant simulation (for each joint)
def simulate_plant(current_angle, control_output, dt):
    # Simple integration to update angle
    return current_angle + control_output * dt

def main():

    # Initialize Board Helper
    helper = bhf.BoardHelper()

    # Initialize System Variables
    dt = 0.01       # Time step [s]
    total_time = 5  # Total simulation time [s]
    num_steps = int(total_time / dt)
    
    # IK and trajectory parameters (from the paper)
    params = {
        'foot_offset': 0.06,
        'l1': 0.0936,
        'l2': 0.18166,
        'l3': 0
    }
    T = 0.4           # Gait cycle period [s]
    step_length = 0.12  # Step length [m]
    hd = 0.26         # Foot height during support phase [m]
    fc = 0.06         # Foot clearance during swing phase [m]
    
    # Define phase offsets for trot gait:
    phases = {
        "FL": 0.0,
        "FR": 0.5,
        "BL": 0.5,
        "BR": 0.0
    }
    
    # Initialize current angles for each joint of each leg (in degrees)
    # For simplicity, we start at 0 for all joints.
    current_angles = {
        "FL": {"joint1": 0.0, "joint2": 0.0},
        "FR": {"joint1": 0.0, "joint2": 0.0},
        "BL": {"joint1": 0.0, "joint2": 0.0},
        "BR": {"joint1": 0.0, "joint2": 0.0}
    }
    
    # Create PID controllers for each joint
    Kp, Ki, Kd = 5.5, 1.4, 0.34
    pid_controllers = {}
    for leg in ["FL", "FR", "BL", "BR"]:
        pid_controllers[leg] = {
            "joint1": PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0)),
            "joint2": PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0))
        }
    
    # Logging for plotting (example for joint1 of FL)
    time_history = []
    desired_FL_joint1 = []
    measured_FL_joint1 = []
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        # For each leg, process the control loop:
        for leg in ["FL", "FR", "BL", "BR"]:
            # 1. Compute desired foot position for this leg (in its local frame)
            desired_foot = foot_trajectory(t, T, phases[leg], step_length, hd, fc)
            
            # 2. Compute desired joint angles via IK for this leg
            theta1_des, theta2_des = inverse_kinematics(desired_foot[0], desired_foot[1], params)
            
            # 3. Update PID setpoints for both joints
            pid_controllers[leg]["joint1"].setpoint = theta1_des
            pid_controllers[leg]["joint2"].setpoint = theta2_des
            
            # 4. Read current measurements (simulate or from sensor) for each joint
            measured_joint1 = current_angles[leg]["joint1"]
            measured_joint2 = current_angles[leg]["joint2"]
            
            # 5. Compute control outputs using PID controllers
            control_output1 = pid_controllers[leg]["joint1"].update(measured_joint1, dt)
            control_output2 = pid_controllers[leg]["joint2"].update(measured_joint2, dt)
            
            # 6. Update the simulated plant for each joint
            current_angles[leg]["joint1"] = simulate_plant(measured_joint1, control_output1, dt)
            current_angles[leg]["joint2"] = simulate_plant(measured_joint2, control_output2, dt)
            
            #print(current_angles[leg])
            # Optionally, add disturbances per leg here if desired.
            
            # Log data (for FL joint1 as an example)
            if leg == "FL":
                desired_FL_joint1.append(theta1_des)
                measured_FL_joint1.append(measured_joint1)

            # Send data to servos
            print(f"theta1_des: {theta1_des}\t theta2_des: {theta2_des}")
            # helper.set_servo_positions(dt, [1, 2], [-theta1_des, -theta2_des])
            helper.set_servo_positions(dt, [4, 3], [10, 10])
        
        time_history.append(t)
        elapsed = time.time() - start_time
        if t > elapsed:
            time.sleep(t - elapsed)
    
    # Plot results for one joint of one leg (e.g., FL joint1)
    plt.figure()
    plt.plot(time_history, desired_FL_joint1, label="Desired FL Joint1")
    plt.plot(time_history, measured_FL_joint1, label="Measured FL Joint1")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("FL Joint1 Tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()