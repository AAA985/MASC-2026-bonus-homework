#!/usr/bin/env python3
"""
Formation scheduler — arrival-detection state machine for SYSU 10-drone swarm.

State machine
─────────────
  INIT
    → HOLD_S_INIT  : show S at initial positions for hold_secs
    → FLYING_TO_Y  : all drones fly to their Y-slot targets; wait for last arrival
    → HOLD_Y       : hold_secs at Y formation
    → FLYING_TO_S2 : fly to S-slot targets at obstacle right-center
    → HOLD_S2      : hold_secs
    → FLYING_TO_U  : fly to U-slot targets right of obstacles
    → HOLD_U       : hold forever

Per-drone target = zone_center + swarm_scale * formation_offset[drone_id]
This mirrors the FSM:  end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos

The 3-second clock starts only AFTER all 10 drones are within arrival_dist of
their individual targets.  During flight the FSM's continuous replanning keeps
obstacle avoidance active at all times.
"""

import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# ── Formation type IDs (must match FORMATION_TYPE enum in poly_traj_optimizer.h) ──
FORMATION_S = 2
FORMATION_Y = 3
FORMATION_U = 4

SWARM_SCALE = 1.0   # must match global_goal/swarm_scale in sysu_formation.yaml
NUM_DRONES  = 10

# Formation offsets (drone_id → [dx, dy, dz])
# Must be identical to setDesiredFormation() cases in poly_traj_optimizer.h
S_OFFSETS = [
    ( 1.5,  3.0, 0), ( 0.0,  3.0, 0), (-1.5,  2.0, 0), (-1.5,  1.0, 0), (-0.5,  0.0, 0),
    ( 0.5,  0.0, 0), ( 1.5, -1.0, 0), ( 1.5, -2.0, 0), ( 0.0, -3.0, 0), (-1.5, -3.0, 0),
]
Y_OFFSETS = [
    (-3.0,  3.0, 0), (-2.0,  2.0, 0), (-1.0,  1.0, 0), ( 3.0,  3.0, 0), ( 2.0,  2.0, 0),
    ( 1.0,  1.0, 0), ( 0.0,  0.0, 0), ( 0.0, -1.0, 0), ( 0.0, -2.0, 0), ( 0.0, -3.0, 0),
]
U_OFFSETS = [
    (-2.0,  3.0, 0), (-2.0,  1.0, 0), (-2.0, -1.0, 0), (-1.5, -3.0, 0), (-0.5, -3.0, 0),
    ( 0.5, -3.0, 0), ( 1.5, -3.0, 0), ( 2.0, -1.0, 0), ( 2.0,  1.0, 0), ( 2.0,  3.0, 0),
]


def make_goal(x, y, z):
    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = 'world'
    g.pose.position.x = float(x)
    g.pose.position.y = float(y)
    g.pose.position.z = float(z)
    g.pose.orientation.w = 1.0
    return g


def targets_for(cx, cy, cz, offsets):
    """Compute per-drone absolute target positions: center + scale * offset."""
    return [
        np.array([cx + SWARM_SCALE * dx,
                  cy + SWARM_SCALE * dy,
                  cz + SWARM_SCALE * dz])
        for dx, dy, dz in offsets
    ]


class FormationScheduler:

    def __init__(self):
        rospy.init_node('formation_scheduler', anonymous=False)

        self.pub_type = rospy.Publisher(
            '/formation_type', Int32, queue_size=1, latch=True)
        self.pub_goal = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

        # ── Timing / detection params ──
        self.init_delay   = rospy.get_param('~init_delay',   5.0)   # s
        self.hold_secs    = rospy.get_param('~hold_secs',    3.0)   # s per formation
        self.arrival_dist = rospy.get_param('~arrival_dist', 1.5)   # m, per-drone threshold
        self.fly_timeout  = rospy.get_param('~fly_timeout',  90.0)  # s before giving up

        # ── Zone centers ──
        self.s_cx  = rospy.get_param('~s_init_zone_x',  25.0)
        self.s_cy  = rospy.get_param('~s_init_zone_y',   0.0)
        self.s_cz  = rospy.get_param('~s_init_zone_z',   0.5)

        self.y_cx  = rospy.get_param('~y_zone_x',   5.0)
        self.y_cy  = rospy.get_param('~y_zone_y',   0.0)
        self.y_cz  = rospy.get_param('~y_zone_z',   0.5)

        self.s2_cx = rospy.get_param('~s2_zone_x', -5.0)
        self.s2_cy = rospy.get_param('~s2_zone_y',  0.0)
        self.s2_cz = rospy.get_param('~s2_zone_z',  0.5)

        self.u_cx  = rospy.get_param('~u_zone_x', -22.0)
        self.u_cy  = rospy.get_param('~u_zone_y',   0.0)
        self.u_cz  = rospy.get_param('~u_zone_z',   0.5)

        # ── Per-drone position cache (updated by odom callbacks) ──
        self._pos = [None] * NUM_DRONES
        for i in range(NUM_DRONES):
            rospy.Subscriber(
                f'/drone_{i}_visual_slam/odom', Odometry,
                lambda msg, idx=i: self._odom_cb(msg, idx))

    # ── Odom callback ──────────────────────────────────────────────────────

    def _odom_cb(self, msg, i):
        p = msg.pose.pose.position
        self._pos[i] = np.array([p.x, p.y, p.z])

    # ── Query helpers ──────────────────────────────────────────────────────

    def _all_have_odom(self):
        return all(p is not None for p in self._pos)

    def _count_arrived(self, targets):
        n = 0
        for i, tgt in enumerate(targets):
            if self._pos[i] is not None:
                if np.linalg.norm(self._pos[i] - tgt) <= self.arrival_dist:
                    n += 1
        return n

    # ── State transitions ──────────────────────────────────────────────────

    def _send_targets(self, ftype, cx, cy, cz):
        """Publish formation type then central goal (0.15 s apart so
        formationTypeCallback updates relative_pos before goal arrives)."""
        self.pub_type.publish(Int32(data=ftype))
        rospy.sleep(0.15)
        self.pub_goal.publish(make_goal(cx, cy, cz))

    def _wait_all_arrived(self, label, targets):
        """Block until all NUM_DRONES are within arrival_dist of their targets,
        or fly_timeout expires.  Returns True if all arrived."""
        deadline = rospy.Time.now() + rospy.Duration(self.fly_timeout)
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            n = self._count_arrived(targets)
            rospy.loginfo_throttle(
                5.0,
                "[Scheduler] %s: %d/%d drones within %.1f m of target",
                label, n, NUM_DRONES, self.arrival_dist)
            if n == NUM_DRONES:
                rospy.loginfo("[Scheduler] All %d drones formed %s",
                              NUM_DRONES, label)
                return True
            if rospy.Time.now() > deadline:
                rospy.logwarn(
                    "[Scheduler] Timeout (%ds) for %s — %d/%d arrived, proceeding",
                    int(self.fly_timeout), label, n, NUM_DRONES)
                return False
            rate.sleep()
        return False

    def _hold(self, label):
        """Hold the current formation for hold_secs.
        EGO-Planner's continuous replanning keeps obstacle avoidance active."""
        rospy.loginfo("[Scheduler] Holding %s for %.1f s ...", label, self.hold_secs)
        rospy.sleep(self.hold_secs)
        rospy.loginfo("[Scheduler] %s hold complete", label)

    # ── Main loop ──────────────────────────────────────────────────────────

    def run(self):
        # ── INIT ──────────────────────────────────────────────────────────
        rospy.loginfo("[Scheduler] Waiting %.1f s for drones to spawn ...",
                      self.init_delay)
        rospy.sleep(self.init_delay)

        rospy.loginfo("[Scheduler] Waiting for all %d drones to publish odom ...",
                      NUM_DRONES)
        poll = rospy.Rate(5.0)
        while not rospy.is_shutdown() and not self._all_have_odom():
            poll.sleep()
        rospy.loginfo("[Scheduler] All drones have odom — starting state machine")

        # ── HOLD_S_INIT  (drones are already at S positions) ──────────────
        s_tgts = targets_for(self.s_cx, self.s_cy, self.s_cz, S_OFFSETS)
        rospy.loginfo("[Scheduler] Phase S-init: center=(%.1f,%.1f,%.1f)",
                      self.s_cx, self.s_cy, self.s_cz)
        self._send_targets(FORMATION_S, self.s_cx, self.s_cy, self.s_cz)
        self._wait_all_arrived("S-init", s_tgts)   # should be immediate
        self._hold("S-init")

        # ── FLYING_TO_Y → HOLD_Y ──────────────────────────────────────────
        y_tgts = targets_for(self.y_cx, self.y_cy, self.y_cz, Y_OFFSETS)
        rospy.loginfo("[Scheduler] Phase Y: center=(%.1f,%.1f,%.1f)",
                      self.y_cx, self.y_cy, self.y_cz)
        self._send_targets(FORMATION_Y, self.y_cx, self.y_cy, self.y_cz)
        self._wait_all_arrived("Y", y_tgts)
        self._hold("Y")

        # ── FLYING_TO_S2 → HOLD_S2 ────────────────────────────────────────
        s2_tgts = targets_for(self.s2_cx, self.s2_cy, self.s2_cz, S_OFFSETS)
        rospy.loginfo("[Scheduler] Phase S2: center=(%.1f,%.1f,%.1f)",
                      self.s2_cx, self.s2_cy, self.s2_cz)
        self._send_targets(FORMATION_S, self.s2_cx, self.s2_cy, self.s2_cz)
        self._wait_all_arrived("S2", s2_tgts)
        self._hold("S2")

        # ── FLYING_TO_U → HOLD_U (forever) ────────────────────────────────
        u_tgts = targets_for(self.u_cx, self.u_cy, self.u_cz, U_OFFSETS)
        rospy.loginfo("[Scheduler] Phase U: center=(%.1f,%.1f,%.1f)",
                      self.u_cx, self.u_cy, self.u_cz)
        self._send_targets(FORMATION_U, self.u_cx, self.u_cy, self.u_cz)
        self._wait_all_arrived("U", u_tgts)
        rospy.loginfo("[Scheduler] U formation complete — holding indefinitely")
        rospy.spin()


if __name__ == '__main__':
    try:
        FormationScheduler().run()
    except rospy.ROSInterruptException:
        pass
