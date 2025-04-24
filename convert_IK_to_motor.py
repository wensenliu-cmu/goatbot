def convert_IK_to_motor(IK_angle, side):
    
    if side.lower() == 'left':
        alpha1 = (180 + IK_angle) % 360
        motor_angle = (120 + alpha1) % 360
    elif side.lower() == 'right':
        alpha2 = (90 + IK_angle) % 360
        motor_angle = (30 + alpha2) % 360
    else:
        raise ValueError("Side must be 'left' or 'right'")
    
    # Clamp the result to the motor's allowed range (0 to 240 degrees)
    motor_angle = max(0, min(motor_angle, 240))
    
    return motor_angle 