#!/usr/bin/env python3
"""
plot_joints.py — Real-time plot of joint angles for Double Inverted Pendulum

Subscribes to /double_inv_pend/joint_states and plots:
  - theta1 (lower rod angle)
  - theta2 (upper rod angle)

Run:
    $ rosrun double_inv_pend plot_joints.py
"""

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import JointState
import math
import collections

# ── Config ──────────────────────────────────────────────────────────
WINDOW_SEC  = 10.0          # seconds of history to show
MAX_POINTS  = 1000          # ring buffer size

# ── Shared state ────────────────────────────────────────────────────
times  = collections.deque(maxlen=MAX_POINTS)
theta1 = collections.deque(maxlen=MAX_POINTS)
theta2 = collections.deque(maxlen=MAX_POINTS)
t0     = None


def joint_state_cb(msg: JointState):
    global t0
    try:
        i1 = msg.name.index('joint1')
        i2 = msg.name.index('joint2')
    except ValueError:
        return

    now = rospy.get_time()
    if t0 is None:
        t0 = now

    times.append(now - t0)
    theta1.append(math.degrees(msg.position[i1]))
    theta2.append(math.degrees(msg.position[i2]))


def animate(frame, ax1, ax2, line1, line2):
    if not times:
        return line1, line2

    t  = list(times)
    y1 = list(theta1)
    y2 = list(theta2)

    # Sliding window
    t_now = t[-1]
    t_min = t_now - WINDOW_SEC

    line1.set_data(t, y1)
    line2.set_data(t, y2)

    for ax in (ax1, ax2):
        ax.set_xlim(max(0, t_min), t_now + 0.5)

    ax1.set_ylim(min(y1 + [-5]) - 2, max(y1 + [5]) + 2)
    ax2.set_ylim(min(y2 + [-5]) - 2, max(y2 + [5]) + 2)

    return line1, line2


def main():
    rospy.init_node('joint_angle_plotter', anonymous=True)
    rospy.Subscriber('/double_inv_pend/joint_states', JointState, joint_state_cb)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Double Inverted Pendulum — Joint Angles', fontsize=14, fontweight='bold')

    line1, = ax1.plot([], [], 'b-', linewidth=1.5, label='θ1 (lower rod)')
    line2, = ax2.plot([], [], 'g-', linewidth=1.5, label='θ2 (upper rod)')

    for ax, label in zip((ax1, ax2), ('θ1 (deg)', 'θ2 (deg)')):
        ax.axhline(0, color='r', linestyle='--', linewidth=0.8, alpha=0.6, label='Setpoint (0°)')
        ax.set_ylabel(label)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    ax2.set_xlabel('Time (s)')

    ani = animation.FuncAnimation(
        fig, animate,
        fargs=(ax1, ax2, line1, line2),
        interval=50,        # 20 Hz refresh
        blit=False
    )

    plt.tight_layout()
    plt.show()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
