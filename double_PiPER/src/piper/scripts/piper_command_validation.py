"""Pure follower command validation. No ROS, SDK connections or device I/O.

Normal following uses directional clipping without fixed joint-angle bounds.
Saved-pose validation keeps the read-only ``get_joint_limits('j1' ... 'j6')``
callable and each driver's diagnostic wording/validation order.
"""
import math


def _sdk_integer(value, label):
    """Check the encoded value before any SDK call can send a partial command."""
    if not math.isfinite(value):
        raise ValueError("%s SDK integer conversion is not finite" % label)
    encoded = round(value)
    if not -(2**31) <= encoded <= 2**31-1:
        raise ValueError("%s outside SDK signed int32 range" % label)
    return encoded


def prepare_follow_command(positions):
    """Copy/normalize a rad/m leader command, returning SDK-quantized rad/m.

    Only J2's negative side, J3's positive side and gripper width are clipped.
    This policy is for normal following, never saved-pose/automatic returns.
    Raw inputs must fit the SDK integer representation even when clipping
    would hide an extreme value. No SDK, firmware or mechanical limit changes.
    """
    values = parse_follow_command(positions)
    # Validate every raw component before clipping can mask numeric corruption.
    for index, value in enumerate(values[:6], 1):
        _sdk_integer(math.degrees(value)*1000, "joint%d" % index)
    _sdk_integer(values[6]*1e6, "gripper")
    values[1] = max(values[1], 0.0)
    values[2] = min(values[2], 0.0)
    gripper = clip_gripper(values[6])
    joints = [math.radians(_sdk_integer(math.degrees(value)*1000, "joint%d" % index)/1000)
              for index, value in enumerate(values[:6], 1)]
    gripper = _sdk_integer(gripper*1e6, "gripper")/1e6
    return joints, gripper


def parse_follow_command(positions):
    """Return a new seven-value list; reject invalid dimensions and NaN/Inf."""
    if len(positions) != 7:
        raise ValueError("Leader command must contain seven values")
    values = [float(value) for value in positions]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Non-finite leader values: %s" % values)
    return values


def clip_gripper(width):
    """Clip a finite follow-command opening to [0, 0.1] metres."""
    width = float(width)
    if not math.isfinite(width):
        raise ValueError("Non-finite gripper value: %s" % width)
    return max(0.0, min(0.1, width))


def validate_joint_limits(joints, get_joint_limits, zero_angle_tolerance=0.05,
                          *, error_style="single"):
    """Copy/validate six joints, allowing only the existing J2/J3 zero margin."""
    if not isinstance(joints, (list, tuple)) or len(joints) != 6:
        raise ValueError("Six joint angles required")
    values = [float(value) for value in joints]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Joint positions contain non-finite values")
    for index, joint in enumerate(values, start=1):
        lower, upper = get_joint_limits("j%d" % index)
        # Exact equality, as in both existing drivers. Do not offset targets.
        if index == 2 and lower == 0:
            lower -= zero_angle_tolerance
        if index == 3 and upper == 0:
            upper += zero_angle_tolerance
        if not lower <= joint <= upper:
            if error_style == "dual":
                raise ValueError("joint%d outside command limits [%.6f, %.6f] rad" %
                                 (index, lower, upper))
            raise ValueError("joint%d %.6f rad outside official limit [%.6f, %.6f]" %
                             (index, joint, lower, upper))
    return values


def _validate_pose_gripper(gripper, error_style):
    if not 0.0 <= gripper <= 0.1:
        if error_style == "dual":
            raise ValueError("Gripper %.3f mm outside [0, 100] mm" % (gripper * 1000))
        raise ValueError("gripper %.6f m outside [0.0, 0.1]" % gripper)


def validate_pose(value, get_joint_limits, zero_angle_tolerance=0.05,
                  *, error_style="single"):
    """Validate the existing pose mapping without clipping or mutating it.

    Preserve conversion/error order: the single driver checks joint bounds
    before gripper bounds; the dual driver checks gripper bounds first.
    """
    dual = error_style == "dual"
    if not isinstance(value, dict):
        raise ValueError("Pose must be a mapping" if dual else "initial pose YAML must be a mapping")
    joints = value.get("joint_positions", value.get("joints"))
    gripper = value.get("gripper")
    if not isinstance(joints, (list, tuple)) or len(joints) != 6:
        raise ValueError("Six joint angles required" if dual else
                         "initial pose must contain six 'joint_positions' values")
    joints, gripper = [float(item) for item in joints], float(gripper)
    if not all(math.isfinite(item) for item in joints + [gripper]):
        if dual:
            raise ValueError("Non-finite joint/gripper values: joints=%s gripper=%s" % (joints, gripper))
        raise ValueError("initial pose contains non-finite values")
    if dual:
        _validate_pose_gripper(gripper, error_style)
    joints = validate_joint_limits(joints, get_joint_limits, zero_angle_tolerance,
                                   error_style=error_style)
    if not dual:
        _validate_pose_gripper(gripper, error_style)
    return joints, gripper


def prepare_sdk_command(joints, gripper, pose_validator):
    """Validate, round at SDK resolution, then revalidate without clipping.

    ``pose_validator`` is the driver's pure pose-validation wrapper. Keeping
    that callback preserves its error reporting and subclass call contract.
    Python's existing round() order (including ties) is retained exactly.
    """
    joints, gripper = pose_validator({"joint_positions": joints, "gripper": gripper})
    joints = [math.radians(round(math.degrees(value) * 1000) / 1000) for value in joints]
    gripper = round(gripper * 1e6) / 1e6
    pose_validator({"joint_positions": joints, "gripper": gripper})
    return joints, gripper
