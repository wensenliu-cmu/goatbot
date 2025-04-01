# ik.py
import math

def inverse_kinematics(x, y, params):
    """
    Compute inverse kinematics for one leg using the 5-bar linkage model.
    
    Args:
        x (float): Desired x position of the foot (in leg's local frame).
        y (float): Desired y position of the foot.
        params (dict): A dictionary containing:
            'foot_offset' (float): Foot offset (l), e.g., 0.06 m.
            'l1' (float): Upper link length, e.g., 0.0936 m.
            'l2' (float): Lower link length, e.g., 0.18166 m.
            'l3' (float): Additional lower link (set to 0 if identical).
    
    Returns:
        tuple: (theta1_deg, theta2_deg) where each angle is in degrees.
    """
    # Effective lower link for branch 1
    lt = params['l2'] + params['l3']
    
    # ---------- Branch 1: Compute theta1 ----------
    a = 2 * x * lt
    b = 2 * y * lt
    c = params['l1']**2 - lt**2 - x**2 - y**2
    
    # Clamp argument to [-1,1]
    arg1 = -c / math.sqrt(a**2 + b**2)
    arg1 = max(min(arg1, 1.0), -1.0)
    
    theta4 = math.atan2(y, x) + math.acos(arg1)
    theta1 = math.atan2(y - lt * math.sin(theta4), x - lt * math.cos(theta4))
    
    # ---------- Branch 2: Compute theta2 ----------
    xC = x - params['l3'] * math.cos(theta4)
    yC = y - params['l3'] * math.sin(theta4)
    xn = xC - params['foot_offset']
    yn = yC
    m = 2 * xn * params['l2']
    n = 2 * yn * params['l2']
    p = params['l1']**2 - params['l2']**2 - xn**2 - yn**2
    
    arg2 = -p / math.sqrt(m**2 + n**2)
    arg2 = max(min(arg2, 1.0), -1.0)
    
    theta3 = math.atan2(xn, yn) - math.acos(arg2)
    theta2 = math.atan2(yn - params['l2'] * math.sin(theta3), xn - params['l2'] * math.cos(theta3))
    
    # Convert to degrees before returning
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    
    return theta1_deg, theta2_deg