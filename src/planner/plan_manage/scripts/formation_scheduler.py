#!/usr/bin/env python3
"""
Formation scheduler for SYSU drone swarm task.
Sends timed formation change commands to all drones via /formation_type topic.
Sequence: S(2) -> Y(3) -> S(2) -> U(4), each formation held for 3 seconds.
"""

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

FORMATION_NONE    = 0
FORMATION_HEXAGON = 1
FORMATION_S       = 2
FORMATION_Y       = 3
FORMATION_U       = 4

HOLD_DURATION = 3.0   # seconds per formation

def main():
    rospy.init_node('formation_scheduler', anonymous=False)

    pub_type = rospy.Publisher('/formation_type', Int32, queue_size=1, latch=True)
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    init_delay = rospy.get_param('~init_delay', 3.0)
    central_x  = rospy.get_param('~central_x',  26.0)
    central_y  = rospy.get_param('~central_y',   0.0)
    central_z  = rospy.get_param('~central_z',   0.5)

    rospy.loginfo("[Scheduler] Waiting %.1f s for drones to initialize...", init_delay)
    rospy.sleep(init_delay)

    # Send formation goal (central position for all drones)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'world'
    goal.pose.position.x = central_x
    goal.pose.position.y = central_y
    goal.pose.position.z = central_z
    goal.pose.orientation.w = 1.0
    pub_goal.publish(goal)
    rospy.loginfo("[Scheduler] Goal sent: (%.1f, %.1f, %.1f)", central_x, central_y, central_z)

    rospy.sleep(0.5)

    # Formation sequence: S -> Y -> S -> U
    sequence = [
        (FORMATION_S, "S"),
        (FORMATION_Y, "Y"),
        (FORMATION_S, "S"),
        (FORMATION_U, "U"),
    ]

    for ftype, fname in sequence:
        if rospy.is_shutdown():
            break
        msg = Int32(data=ftype)
        pub_type.publish(msg)
        rospy.loginfo("[Scheduler] Formation: %s (type=%d)", fname, ftype)
        rospy.sleep(HOLD_DURATION)

    # Hold final U formation
    rospy.loginfo("[Scheduler] Sequence complete. Holding U formation.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
