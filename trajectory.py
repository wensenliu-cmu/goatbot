# trajectory.py
import math

def sin_trajectory(t, T, phase, step_length, hd, fc):
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
        y = -hd
        
        #y = -hd  # support phase: foot at constant height
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
"""
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # Test dynamic gait trajectory over one cycle
    T = 0.4            # Gait cycle period [s]
    step_length = 0.12 # Step length [m]
    hd = 0.26          # Support phase foot height [m]
    fc = 0.06          # Foot clearance [m]
    phase = 0.0        # For example, phase offset for one leg
    num_samples = 4000
    
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
    """
    
import numpy as np

def create_trajectory(num_points=5000, cycles=3, fixed_x=0.03):
   
    trajectory = []  # List to store (x, y) coordinate pairs

    for _ in range(cycles):
        # Create a downward movement from -5 to -10
        y_down = np.linspace(-0.1947, -0.097, num_points)
        # Create an upward movement from -10 back to -5
        y_up = np.linspace(-0.097, -0.1947, num_points)
        
        # Append downward and upward movements to the trajectory
        for y in y_down:
            trajectory.append((fixed_x, y))
        for y in y_up:
            trajectory.append((fixed_x, y))
    
    return trajectory

# Example usage:
if __name__ == "__main__":
    traj = create_trajectory()
    # Print the trajectory coordinates
    for coord in traj:
        print(coord)


def manual_traj(t, T, num_samples):
    """
    Computes the manual foot trajectory as a function of time t.
    
    The trajectory is composed of five segments that form a closed loop:
    
      Segment A (Upper Left Arc):
          A counterclockwise circular arc centered at (0,0) with radius r_top = 0.0936,
          starting at (0, -r_top) (at -90°) and rotating to approximately (-0.0485, -0.084)
          (at -120°).
    
      Segment B (Left Linear Segment):
          A straight line from the end of Segment A to (-0.13763, -0.2584).
    
      Segment C (Bottom Circular Arc):
          A clockwise circular arc connecting (-0.13763, -0.2584) to (0.13763, -0.2584).
          The circle is centered at (0,0) with a radius computed from the endpoint coordinates.
    
      Segment D (Right Linear Segment):
          A straight line from (0.13763, -0.2584) to (0.0485, -0.084).
    
      Segment E (Upper Right Arc):
          A counterclockwise circular arc (again with radius r_top) from (0.0485, -0.084)
          back to (0, -r_top). This arc uses angles from -60° to -90°.
    
    The full cycle is built at an internal (native) resolution and then re-sampled to
    have exactly num_samples points per cycle. The function then returns the coordinate corresponding
    to the phase of t, where the phase is computed as (t mod T) / T.
    
    Parameters:
        t (float): Current time (seconds).
        T (float): Gait cycle period (seconds).
        num_samples (int): Number of points (resolution) in one complete cycle.
    
    Returns:
        coord (np.ndarray): A 1D array [x, y] representing the foot position at time t.
    """
    # --- Define native sample counts for each segment ---
    N_arc = 50         # for each upper circular arc (Segments A and E)
    N_line = 50        # for each linear segment (Segments B and D)
    N_bottom_arc = 100 # for the bottom arc (Segment C)
    
    # ---------------------------
    # Segment A: Upper Left Arc (CCW)
    # ---------------------------
    r_top = 0.0936
    theta_A_start = -np.pi/2       # -90° (starting at (0, -r_top))
    theta_A_end   = -2*np.pi/3      # -120°
    theta_A = np.linspace(theta_A_start, theta_A_end, N_arc, endpoint=False)
    x_A = r_top * np.cos(theta_A)
    y_A = r_top * np.sin(theta_A)
    
    # ---------------------------
    # Segment B: Left Linear Segment
    # ---------------------------
    # End of Segment A
    pt_A_end = np.array([x_A[-1], y_A[-1]])
    pt_B_end = np.array([-0.13763, -0.2584])
    t_B = np.linspace(0, 1, N_line, endpoint=False)
    x_B = pt_A_end[0] + (pt_B_end[0] - pt_A_end[0]) * t_B
    y_B = pt_A_end[1] + (pt_B_end[1] - pt_A_end[1]) * t_B
    
    # ---------------------------
    # Segment C: Bottom Circular Arc (CW)
    # ---------------------------
    # Endpoints for the bottom arc:
    # Left endpoint: (-0.13763, -0.2584)
    # Right endpoint: (0.13763, -0.2584)
    r_bottom = np.sqrt((-0.13763)**2 + (-0.2584)**2)
    theta_left = np.arctan2(-0.2584, -0.13763)
    theta_right = np.arctan2(-0.2584,  0.13763)
    # For a continuous clockwise arc, ensure that theta_left is less than theta_right.
    if theta_left > theta_right:
        theta_left -= 2 * np.pi
    theta_C = np.linspace(theta_right, theta_left, N_bottom_arc, endpoint=False)
    x_C = r_bottom * np.cos(theta_C)
    y_C = r_bottom * np.sin(theta_C)
    
    # ---------------------------
    # Segment D: Right Linear Segment
    # ---------------------------
    # From (0.13763, -0.2584) to (0.0485, -0.084)
    pt_C_start = np.array([0.13763, -0.2584])
    pt_D_end = np.array([0.0485, -0.084])
    t_D = np.linspace(0, 1, N_line, endpoint=False)
    x_D = pt_C_start[0] + (pt_D_end[0] - pt_C_start[0]) * t_D
    y_D = pt_C_start[1] + (pt_D_end[1] - pt_C_start[1]) * t_D
    
    # ---------------------------
    # Segment E: Upper Right Arc (CCW)
    # ---------------------------
    # From (0.0485, -0.084) back to (0, -r_top)
    theta_E_start = -np.pi/3       # -60°
    theta_E_end   = -np.pi/2       # -90°
    theta_E = np.linspace(theta_E_start, theta_E_end, N_arc, endpoint=False)
    x_E = r_top * np.cos(theta_E)
    y_E = r_top * np.sin(theta_E)
    
    # ---------------------------
    # Concatenate segments (the native trajectory)
    # ---------------------------
    x_total = np.concatenate([x_A, x_B, x_C, x_D, x_E])
    y_total = np.concatenate([y_A, y_B, y_C, y_D, y_E])
    
    # --- Re-sample to exactly num_samples points along the cycle ---
    # First, build a parameter array corresponding to the native trajectory (assumed uniformly spaced)
    N_total = len(x_total)
    s_native = np.linspace(0, 1, N_total, endpoint=False)
    s_resampled = np.linspace(0, 1, num_samples, endpoint=False)
    
    # Interpolate each coordinate to the new resolution.
    x_resampled = np.interp(s_resampled, s_native, x_total)
    y_resampled = np.interp(s_resampled, s_native, y_total)
    traj_resampled = np.vstack((x_resampled, y_resampled)).T  # shape (num_samples, 2)
    
    # --- Determine the current phase based on time t and cycle period T ---
    phase = (t % T) / T  # normalized phase in [0, 1)
    index = int(phase * num_samples)  # map phase to an index (0 <= index < num_samples)
    
    return x_resampled, y_resampled

    # Optional: Add a main block to test these functions.

    import math

def semiElliptical_trajectory(t, T, phase=0.0,
                    x_start=-0.01, step_length=0.10,
                    stance_y=-0.20, clearance_upper=0.05,
                    clearance_lower=0.01):
    """
    Compute a single gait foot position [x, y] at time t over period T.

    Parameters:
    - t: current time
    - T: gait cycle period
    - phase: phase offset in [0,1)
    - x_start: starting x position (e.g. -0.01 m)
    - step_length: horizontal excursion (end at x_start + step_length)
    - stance_y: vertical position during stance (foot on ground, e.g. -0.20 m)
    - clearance_upper: max foot height above stance_y during swing (e.g. 0.05 m)
    - clearance_lower: depth of rounding below stance_y during stance (e.g. 0.01 m)
    """
    # Normalize phase
    p = ((t / T + phase) % 1.0)
    mid = x_start + step_length / 2
    a = step_length / 2

    if p < 0.5:
        # Swing phase: upper half‑ellipse
        p1 = p * 2          # map [0,0.5)->[0,1)
        phi = math.pi * (1 - p1)  # π→0
        x = mid + a * math.cos(phi)
        y = stance_y + clearance_upper * math.sin(phi)
    else:
        # Stance phase: lower half‑ellipse (rounded)
        p2 = (p - 0.5) * 2  # map [0.5,1)->[0,1)
        phi2 = math.pi * p2  # 0→π
        x = mid + a * math.cos(phi2)
        y = stance_y - clearance_lower * math.sin(phi2)

    return x, y

def sinusoidal_trajectory(t, T, f, x_amp, y_amp, x_offset, y_offset):
    """
    Generate a sinusoidal x,y trajectory over time vector t.

    Parameters
    ----------
    t : array_like
        Time samples (e.g. np.linspace(0, 2*T, N)).
    T : float
        Period of motion in seconds.
    f : float
        Frequency of motion in Hz (typically f = 1/T).
    x_amp : float
        Amplitude of the x‑trajectory.
    y_amp : float
        Amplitude of the y‑trajectory.
    x_offset : float
        Center offset of the x‑trajectory.
    y_offset : float
        Center offset of the y‑trajectory.

    Returns
    -------
    x_traj : ndarray
        x positions at each time in t.
    y_traj : ndarray
        y positions at each time in t.
    """
    t = np.asarray(t)
    # x varies sinusoidally around x_offset
    x_traj = x_offset + x_amp * np.sin(2 * np.pi * f * t)
    # y varies cosinusoidally around y_offset
    y_traj = y_offset + y_amp * np.cos(2 * np.pi * f * t)
    return x_traj, y_traj

    
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # Test dynamic gait trajectory over one cycle
    T = 0.4            # Gait cycle period [s]
    f = 1/T
    step_length = 0.12 # Step length [m]
    hd = 0.26          # Support phase foot height [m]
    fc = 0.06          # Foot clearance [m]
    phase = 0.0        # For example, phase offset for one leg
    num_samples = 4000
    
    # larger gait parameters
    x_amp = 0.08
    y_amp = 0.05
    x_off = 0.0
    y_off = -0.15
    
    t_values = [i * T/100.0 for i in range(101)]
    num_samples = 4000  # high resolution sampling
    t_cycle = np.linspace(0, T, num_samples, endpoint=False)
    #t_cycle = np.linspace(0, 2*T, 200)
    #traj = [semiElliptical_trajectory(
    #                                t, 
    #                                T,
    #                                phase=phase,
    #                                x_start=-0.01,
    #                                step_length=0.10,
    #                                stance_y=-0.20,
    #                                clearance_upper=0.05,
    #                                clearance_lower=0.01) for t in t_cycle]
    traj = [sinusoidal_trajectory(t, T, f, x_amp, y_amp, x_off, y_off) for t in t_cycle]
    xs, ys = zip(*traj)
    
    plt.figure()
    plt.plot(xs, ys, 'o-')
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