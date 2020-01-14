# #!/usr/bin/env python


import argparse
import sys

from copy import copy

import rospy

import actionlib
import numpy as np
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class Trajectory(object):
    def __init__(self, joint_names, action_name):
        self._client = actionlib.SimpleActionClient(
            action_name,
            FollowJointTrajectoryAction,
        )
        self._joint_names = joint_names
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()
        self.duration = 0.0

    def add_point(self, positions, dt):
        self.duration += dt
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(self.duration)

        # Here we add velocities and accelerations
        # v
        if len(self._goal.trajectory.points) > 1:
            point.velocities = list(1.0 * (np.asarray(positions) -\
                        np.asarray(self._goal.trajectory.points[-1].positions)) / dt)
        else:
            point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # a
        if len(self._goal.trajectory.points) > 2:
            point.accelerations = list(1.0 * (np.asarray(point.velocities) - \
                    np.asarray(self._goal.trajectory.points[-1].velocities)) / dt)
        else:
            point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Code ends

        self._goal.trajectory.points.append(point)


    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        print(self._goal)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names


def main():

    #set action name and joint names
    rospy.init_node('Sawyer_joint_trajectory_client')
    joint_names = [ 'right_j' + str(x) for x in range(7)]
    action_name = '/robot/limb/right/follow_joint_trajectory'

    traj = Trajectory(joint_names, action_name)
    
    rospy.on_shutdown(traj.stop)
    
    current_angles = 7*[0.0]
    traj.add_point(current_angles, 0.0)
    traj.add_point([x + 0.1 for x in current_angles], 4.0)
    traj.add_point([x + 0.2 for x in current_angles], 4.0)
    traj.add_point([x + 0.3 for x in current_angles], 4.0)
    traj.add_point([x + 0.4 for x in current_angles], 4.0)
    
    traj.start()
    traj.wait(15.0)

if __name__ == "__main__":
    main()
