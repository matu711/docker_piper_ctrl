#!/usr/bin/python3
"""Select established control adapters; recorder never calls these services."""
import sys
import rospy


def main():
    # Explicit argv keeps driver constructors' original init_node/order intact.
    configuration, role = rospy.myargv(argv=sys.argv)[1:]
    if role == "leader":
        # Byte-identical to the corrected reference's piper_dual_leader.py.
        from piper_ctrl_leader_node_new import PiperLeaderNode
        node = PiperLeaderNode()
    elif configuration == "dual":
        from piper_follower import PiperFollowerNode
        node = PiperFollowerNode()
    else:
        # Preserve single-arm command clipping, fault latch, EE, strict home.
        from piper_ctrl_follower_node import PiperFollowerNode
        node = PiperFollowerNode()
    rospy.spin()
    return node


if __name__ == "__main__":
    main()
