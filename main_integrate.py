# main_integrate.py

import time
import math
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ik import inverse_kinematics
from trajectory import foot_trajectory
from motor_interface import send_motor_command  # real motor interface
from imu_interface import read_imu_data         # real IMU interface

# For simulation/testing, we keep a dummy plant function.
def simulate_plant(current_angle, control_output, dt):
    # A simple integrator model (replace with real sensor feedback later)
    return current_angle + control_output * dt

def main():
    dt = 0.01       # Time step [s]
    total_time = 5  # Total simulation time [s]
    num_steps = int(total_time / dt)
    
    # IK and trajectory parameters
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
    
    # For full quadruped: define phases per leg
    phases = {"FL": 0.0, "FR": 0.5, "BL": 0.5, "BR": 0.0}
    
    # Initialize current angles for one joint per leg (for simplicity, one joint per leg)
    current_angles = {leg: 0.0 for leg in phases.keys()}
    
    # Create PID controllers for each leg (for one joint each for now)
    Kp, Ki, Kd = 5.5, 1.4, 0.34
    pid_controllers = {leg: PIDController(Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0))
                       for leg in phases.keys()}
    
    # Logging for one leg (e.g., FL) for visualization
    time_history = []
    desired_history = []
    measured_history = []
    command_history = []
    
    start_time = time.time()
    for step in range(num_steps):
        t = step * dt
        
        # --- Read IMU Data (for possible body stabilization, not used here directly) ---
        imu_data = read_imu_data()  # e.g., returns {'roll':0, 'pitch':0, 'yaw':0}
        # Here you might use imu_data to adjust your desired trajectories or compensate for disturbances.
        
        # Process each leg:
        for leg in phases:
            desired_foot = foot_trajectory(t, T, phases[leg], step_length, hd, fc)
            theta1_des, theta2_des = inverse_kinematics(desired_foot[0], desired_foot[1], params)
            # For simplicity, we use theta1_des for our joint control.
            pid_controllers[leg].setpoint = theta1_des
            
            # In a real system, you'd read the actual joint angle from sensors.
            # Here we simulate with our dummy plant.
            measured_angle = current_angles[leg]
            
            control_output = pid_controllers[leg].update(measured_angle, dt)
            
            # Send motor command for this leg (replace joint id with actual mapping)
            send_motor_command(leg, control_output)
            
            # Update simulated plant for this leg
            current_angles[leg] = simulate_plant(measured_angle, control_output, dt)
            
            # Log data for FL for example
            if leg == "FL":
                desired_history.append(theta1_des)
                measured_history.append(measured_angle)
                command_history.append(control_output)
        
        time_history.append(t)
        elapsed = time.time() - start_time
        if t > elapsed:
            time.sleep(t - elapsed)
    
    # Plot results for one leg (FL)
    plt.figure(figsize=(10, 8))
    plt.subplot(3,1,1)
    plt.plot(time_history, desired_history, label="Desired Angle")
    plt.plot(time_history, measured_history, label="Measured Angle")
    plt.title("FL Joint Angle Tracking")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3,1,2)
    plt.plot(time_history, command_history, label="PID Output")
    plt.title("FL PID Controller Output")
    plt.ylabel("Control Signal")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3,1,3)
    # Here you can plot some IMU data if desired, for now just a placeholder.
    imu_yaw = [0 for _ in time_history]  # Replace with real IMU values if needed.
    plt.plot(time_history, imu_yaw, label="IMU Yaw")
    plt.title("IMU Data (Yaw)")
    plt.xlabel("Time (s)")
    plt.ylabel("Degrees")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user")