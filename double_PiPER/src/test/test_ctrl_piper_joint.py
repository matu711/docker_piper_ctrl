#!/usr/bin/env python3
"""Explicit opt-in joint/gripper motion check; the default only prints a plan."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'piper/scripts'))
from piper_command_validation import prepare_sdk_command, validate_pose


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', required=True, choices=('can0', 'can1', 'can2', 'can3'))
    parser.add_argument('--execute', action='store_true', help='Connect and execute the displayed motion plan')
    parser.add_argument('--radius', type=float, default=.15, help='J1/J2 sinusoid amplitude [rad], not a Cartesian radius')
    parser.add_argument('--period', type=float, default=2.)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--dt', type=float, default=.01)
    parser.add_argument('--speed', type=int, default=40)
    parser.add_argument('--grip-min', type=float, default=.01)
    parser.add_argument('--grip-max', type=float, default=.1)
    parser.add_argument('--grip-period', type=float, default=2.)
    parser.add_argument('--feedback-timeout', type=float, default=1.)
    parser.add_argument('--enable-timeout', type=float, default=10.)
    args = parser.parse_args(argv)
    for name in ('period', 'duration', 'dt', 'grip_period', 'feedback_timeout', 'enable_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error('%s must be finite and positive' % name)
    if not math.isfinite(args.radius) or args.radius < 0:
        parser.error('--radius must be finite and nonnegative')
    if not 1 <= args.speed <= 100:
        parser.error('--speed must be in 1..100')
    if not all(math.isfinite(v) for v in (args.grip_min, args.grip_max)) or not 0 <= args.grip_min <= args.grip_max <= .1:
        parser.error('gripper bounds must satisfy 0 <= min <= max <= 0.1 m')
    return args


def motion_target(base, elapsed, args):
    joints = list(base)
    angle = 2 * math.pi * elapsed / args.period
    joints[0] += args.radius * math.cos(angle)
    joints[1] += args.radius * math.sin(angle)
    fraction = (math.sin(2 * math.pi * elapsed / args.grip_period) + 1) / 2
    return joints, args.grip_min + fraction * (args.grip_max - args.grip_min)


def require_unused_can(port):
    """Fail closed on existing RX owners; this cannot detect send-only clients."""
    files = list(Path('/proc/net/can').glob('rcvlist_*'))
    if not files:
        raise RuntimeError('Kernel CAN ownership information unavailable; refusing execution')
    for path in files:
        for line in path.read_text().splitlines():
            fields = line.split()
            if fields and fields[0] in (port, 'any'):
                raise RuntimeError('Existing CAN receiver; stop other drivers first: ' + line.strip())


def check_status(interface):
    status = interface.GetArmStatus()
    if (not math.isfinite(status.time_stamp) or status.time_stamp <= 0 or
            int(status.arm_status.err_code) or int(status.arm_status.arm_status) != 0 or
            int(status.arm_status.ctrl_mode) != 1):
        raise RuntimeError('Fresh, error-free CAN-control status required; no automatic teaching-mode recovery')


def run_motion(args, piper_factory, params_factory):
    """Called only after --execute and ownership checks. Factories allow mocks."""
    if not args.execute:
        raise RuntimeError('--execute is required for motion')
    from read_joint_angles import read_updated_state
    piper = piper_factory(args.can)
    interface = None
    try:
        interface = piper.init()
        piper.connect()
        feedback = read_updated_state(piper, timeout=args.feedback_timeout)
        base, last_stamp = list(feedback[0]), float(feedback[1])
        if not math.isfinite(last_stamp) or last_stamp <= 0:
            raise RuntimeError('Invalid joint feedback timestamp')
        check_status(interface)
        params = params_factory()
        # This manual motion test uses strict SDK limits, without widening them.
        def validator(value):
            return validate_pose(value, params.GetJointLimitParam, zero_angle_tolerance=0.)
        validator({'joint_positions': base, 'gripper': args.grip_min})
        for index in range(6):
            low = base[index] - (args.radius if index < 2 else 0.)
            high = base[index] + (args.radius if index < 2 else 0.)
            print('J%d planned range: %.6f .. %.6f rad' % (index+1, low, high), flush=True)
            for value in (low, high):
                target = base.copy()
                target[index] = value
                prepare_sdk_command(target, args.grip_min, validator)
                prepare_sdk_command(target, args.grip_max, validator)
        deadline = time.monotonic() + args.enable_timeout
        while not piper.enable_arm():
            if time.monotonic() >= deadline:
                raise RuntimeError('Motor enable timed out')
            time.sleep(.01)
        piper.enable_gripper()
        interface.ModeCtrl(1, 1, args.speed, 0)
        started = time.monotonic()
        last_progress = 0.
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= args.duration:
                break
            check_status(interface)
            measured = piper.get_joint_states()
            stamp = float(measured[1])
            if not math.isfinite(stamp) or stamp < last_stamp:
                raise RuntimeError('Invalid/regressed joint feedback timestamp')
            if stamp > last_stamp:
                last_stamp, last_progress = stamp, elapsed
            if elapsed - last_progress > args.feedback_timeout:
                raise RuntimeError('Joint feedback stopped updating')
            joints, grip = motion_target(base, elapsed, args)
            joints, grip = prepare_sdk_command(joints, grip, validator)
            piper.move_j(joints, args.speed)
            piper.move_gripper(grip, 1)
            print('CAN:', args.can, 'Joint_Pos:', measured[0],
                  'gripper:', piper.get_gripper_states()[0][0],
                  'End_Pos_Euler:', piper.get_end_pose_euler()[0], flush=True)
            time.sleep(args.dt)
    finally:
        try:
            if interface is not None:
                # Do not disable torque or automatically recover a stopped arm.
                interface.EmergencyStop(1)
        finally:
            disconnect = getattr(piper, 'disconnect', None)
            if callable(disconnect):
                disconnect()


def execute(args):
    if not args.execute:
        raise RuntimeError('--execute is required for CAN connection')
    # Cooperative lock plus a conservative kernel RX-owner check. Other ROS/SDK
    # processes must be stopped; neither check is a system-wide CAN lease.
    with open('/tmp/piper_joint_test_%s.lock' % args.can, 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another joint test owns ' + args.can) from exc
        require_unused_can(args.can)
        from piper_sdk import Piper
        from piper_sdk.piper_param.piper_param_manager import C_PiperParamManager
        run_motion(args, Piper, C_PiperParamManager)


def main(argv=None):
    args = parse_args(argv)
    print(json.dumps(vars(args), sort_keys=True))
    print('Plan: J1=initial+radius*cos(t), J2=initial+radius*sin(t); J3..J6 unchanged.')
    print('Gripper: %.6f .. %.6f m. SDK limits and the full joint envelope are checked before enable.' %
          (args.grip_min, args.grip_max))
    print('Stop all other ROS/SDK CAN owners. Joint limits do not prove collision clearance.', flush=True)
    if not args.execute:
        print('DRY RUN: no CAN connection, enable or command. Add --execute to run.')
        return 0
    execute(args)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        print('ERROR:', exc, file=sys.stderr)
        sys.exit(1)
