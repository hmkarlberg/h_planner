#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import numpy as np
import numpy.linalg as npla

import threading
import time

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class BaxterSim(object):

    def __init__(self):
        # setting scene and robot
        print("\n===== init moveit commander =====\n")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(0.1)

        print("\n===== init rospy node =====\n")
        rospy.init_node('baxter_interface_hampus', anonymous=True)
        rospy.sleep(0.1)

        print("\n===== set robot =====\n")
        self.robot = moveit_commander.RobotCommander()
        rospy.sleep(0.1)

        print("\n===== set scene =====\n")
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.1)

        # designating limbs
        self.r_hand = moveit_commander.MoveGroupCommander('right_hand')
        self.r_hand.set_start_state(moveit_msgs.msg.RobotState())

        self.r_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.r_arm.set_start_state(moveit_msgs.msg.RobotState())

        self.l_hand = moveit_commander.MoveGroupCommander('left_hand')
        self.l_hand.set_start_state(moveit_msgs.msg.RobotState())

        self.l_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.l_arm.set_start_state(moveit_msgs.msg.RobotState())

        self.limbs = {'r_hand':self.r_hand, 'r_arm':self.r_arm, 'l_hand':self.l_hand, 'l_arm':self.l_arm}

        # setting up object and parameters needed
        self.held_object = held_object()
        self.held_object.header.frame_id = 'right_arm'
        self.held_object.pose.orientation.w = 1.0
        self.object_name = 'object'

        self.slide = False
        self.roll = False
        self.pivot = False

        self.push = False



    def set_planning_params(self, limb, time = 1, attempts = 1):
        self.limbs[limb].set_planning_time(time)
        self.limbs[limb].set_num_planning_attempts(attempts)


    def go_to_pose(self, limb, x, y, z, roll, pitch, yaw):

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.limbs[limb].set_start_state_to_current_state()

        self.limbs[limb].set_pose_target(pose_goal)
        self.limbs[limb].go(wait=True)

        self.limbs[limb].stop()
        self.limbs[limb].clear_pose_targets()

    def go_to_position(self, limb, x, y, z):

        self.limbs[limb].set_start_state_to_current_state()

        self.limbs[limb].set_position_target([x,y,z])
        self.limbs[limb].go(wait=True)

        self.limbs[limb].stop()
        self.limbs[limb].clear_pose_targets()

    def go_to_orientation(self, limb, roll, pitch, yaw):

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        self.limbs[limb].set_start_state_to_current_state()

        self.limbs[limb].set_orientation_target(quat)
        self.limbs[limb].go(wait=True)

        self.limbs[limb].stop()
        self.limbs[limb].clear_pose_targets()

    def add_object(self, timeout = 4):
        self.scene.add_box(self.object_name, self.held_object, size = ())
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout = 4):
        grasping_group = 'gripper'
        touch_links = self.robot.get_link_names(group = grasping_group)
        self.scene.attach_box(self.eef_link, box_name, touch_links = touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout = 4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        self.§scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


class HeldObject(geometry_msgs.msg.PoseStamped):

    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

        self.contact_points = {}

        self.sliding = False
        self.rolling = False
        self.pivoting = False

    def add_contact(self, coor, f_dir, f_mag):
        self.contact_points[coor] = ContactPoint(coor, f_dir, f_mag)

    def set_contact_coor(self, point, new_coor):
        self.contact_points[point].coordinate = new_coor

    def set_contact_f_dir(self, point, new_dir):
        self.contact_points[point].force_direction = new_dir

    def set_contact_f_mag(self, pint, new_mag):
        self.contact_points[point].force_magnitude = new_mag

    def set_contact(self, point, new_coor, new_dir, new_mag):
        self.contact_points[point].coordinate = new_coor
        self.contact_points[point].force_direction = new_dir
        self.contact_points[point].force_magnitude = new_mag


class ContactPoint(object):

    def __init__(self, coor, f_dir, f_mag):
        self.coordinate = coor
        self.force_direction = f_dir
        self.force_magnitude = f_mag


























class Tracker(object):

    self.updating = False
    self.positiion = np.zeros(3,1)
    self.orientation = np.zeros(3,1)

    def __init__(self, tracked_object, holder):
        self.tracked_object = tracked_object
        self.holder = holder

    def track_object(self):
        while self.updating == True:
            self.position = self.get_position()
            self.orientation = self.get_orientation()
            self.update_object()

    def update_object(self):
        self.collision_check()

    def collision_check(self):

class Point(object):

    def __init__(self,x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, point):
        return Point(self.x + point.x, self.y + point.y, self.z + point.z,)

    def __sub__(self, point):
        return Point(self.x - point.x, self.y - point.y, self.z - point.z,)












class BaxterScene(object):

class LimbMover(object):

    def __init__(self):

    def set_planner_params(self):

    def go_to_pose(self):

    def go_to_position(self):

    def go_to_orientation(self):




class ObjectMover(object):

    def __init__(self):

    def go_to_position(self):

    def go_to_orientation(self):



def main():

if __name__ == '__main__':
    main()
