import math

def calc_angles(x, y, params, prev_config):
    """
    Compute the two shoulder (motor) angles for a 5‑bar linkage leg in radians.

    Args:
        x (float): desired foot x-position (m) in local frame
        y (float): desired foot y-position (m) in local frame
        l0 (float): horizontal base offset from hip to each motor (m)
        l1 (float): upper link length (m)
        l2 (float): lower link length (m)
        prev_config (str): initial config hint, "outward" or "mixed"

    Returns:
        (shoulder1, shoulder2): tuple of motor angles in radians,
                                or (math.nan, math.nan) if unreachable.
    """

    l0, l1, l2 = params['foot_offset'], params['l1'], params['l2']

    # 1. Base-to-joint angles β using atan2
    #    Left motor base at (-l0,0): β1 = atan2(y, x + l0)
    #    Right base at (+l0,0):       β2 = atan2(y, x - l0)
    beta1 = math.atan2(y, x + l0)
    beta2 = math.atan2(y, x - l0)

    # 2. Law of Cosines to find elbow angles
    #    r1 = distance from left base to foot
    #    r2 = distance from right base to foot
    r1_sq = (x + l0)**2 + y**2
    r2_sq = (x - l0)**2 + y**2
    cos_alpha1 = (r1_sq + l1**2 - l2**2) / (2 * l1 * math.sqrt(r1_sq))
    cos_alpha2 = (r2_sq + l1**2 - l2**2) / (2 * l1 * math.sqrt(r2_sq))

    # 3. Check reachability
    if abs(cos_alpha1) > 1 or abs(cos_alpha2) > 1:
        print(f"Unreachable coordinates: x={x}, y={y}")
        return math.nan, math.nan

    # 4. Compute elbow-up (positive) and elbow-down (negative) angles
    alpha1_up   = math.acos(cos_alpha1)
    alpha1_down = -alpha1_up
    alpha2_up   = math.acos(cos_alpha2)
    alpha2_down = -alpha2_up

    # 5. Two configuration options
    #    Outward: both elbows “up”
    shoulder1_out = beta1 - alpha1_up
    shoulder2_out = beta2 + alpha2_up
    #    Mixed: left up, right down
    shoulder1_mix = beta1 - alpha1_up
    shoulder2_mix = beta2 + alpha2_down

    # 6. Continuity: store last choice on the function object
    if not hasattr(calc_angles, "last_config"):
        calc_angles.last_config = prev_config

    # 7. Select based on previous
    if calc_angles.last_config == "outward":
        shoulder1, shoulder2 = shoulder1_out, shoulder2_out
    else:
        shoulder1, shoulder2 = shoulder1_mix, shoulder2_mix

    # 8. Update last_config for next call
    if abs(shoulder2 - shoulder2_out) < abs(shoulder2 - shoulder2_mix):
        calc_angles.last_config = "outward"
    else:
        calc_angles.last_config = "mixed"

    # 9. Debug print in degrees
    print(f"Selected Angles: S1={math.degrees(shoulder1):.2f}°, S2={math.degrees(shoulder2):.2f}°")

    return shoulder1, shoulder2