#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time

from geometry_msgs.msg import PoseStamped, Pose
import rclpy

from robot_navigator import BasicNavigator

'''
Basic navigation demo to go to pose.
This will block until the navigation task is complete.
To cancel navigation on-going, you need to process in a separate thread.

This is obviously a very simplistic demo, in real use, you wouldn't
block navigation so you could cancel, preempt, receive feedback,
do other things here, etc to make the best use of the ROS actions.
'''
def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = 3.45
    initial_pose.position.y = 2.15
    initial_pose.orientation.z = 1.0
    initial_pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)

    while not navigator.checkStatus():
        feedback = navigator.getFeedback()
        print()
        if (t > 10):
            navigator.cancelNav()

    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################

    # cancel just to make sure nothing is still processing
    navigator.cancelNav()
    exit(0)


if __name__ == '__main__':
    main()
