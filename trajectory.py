# trajectory.py
import math

def foot_trajectory(t, T, phase, step_length, hd, fc):
    """
    Compute the 2D foot position along a semi-elliptical gait trajectory.
    
    Args:
        t (float): Current time [s].
        T (float): Gait cycle period [s].
        phase (float): Phase offset for the leg (e.g., 0 for one leg, 0.5 for its diagonal).
        step_length (float): Total step length [m].
        hd (float): Desired foot height during support phase [m].
        fc (float): Foot clearance during the swing phase [m].
    
    Returns:
        (tuple): (x, y) position of the foot.
    """
    # Normalize time with phase offset
    tau = ((t / T) + phase) % 1.0
    
    # Compute x coordinate (same for both phases)
    x = (step_length / 2.0) * math.cos(2 * math.pi * (1 - tau))
    
    # Compute y coordinate: constant during support phase, sinusoidal during swing phase
    if tau < 0.5:
        y = -hd  # support phase: foot at constant height
    else:
        y = -hd + fc * math.sin(2 * math.pi * (1 - tau))  # swing phase: add clearance
    
    return x, y

def standing_foot(hd=0.26):
    """
    Return a constant foot position for standing.
    
    Args:
        hd (float): Desired foot height during support phase [m]. Defaults to 0.26.
    
    Returns:
        (tuple): (x, y) position for a standing foot.
    """
    # For standing, we can assume the foot remains directly under the hip.
    # Here, x = 0.0 and y = -hd.
    return 0.0, -hd

# Optional: Add a main block to test these functions.
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # Test dynamic gait trajectory over one cycle
    T = 0.4            # Gait cycle period [s]
    step_length = 0.12 # Step length [m]
    hd = 0.26          # Support phase foot height [m]
    fc = 0.06          # Foot clearance [m]
    phase = 0.0        # For example, phase offset for one leg
    
    t_values = [i * T/100.0 for i in range(101)]
    traj = [foot_trajectory(t, T, phase, step_length, hd, fc) for t in t_values]
    xs, ys = zip(*traj)
    
    plt.figure()
    plt.plot(xs, ys, 'o-', label='Gait Trajectory')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Foot Trajectory over One Gait Cycle')
    plt.legend()
    plt.grid(True)
    
    # Test standing foot trajectory
    x_stand, y_stand = standing_foot(hd)
    plt.plot([x_stand], [y_stand], 'rx', markersize=12, label='Standing Position')
    plt.legend()
    plt.show()
    