#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface

import numpy as np
import numpy.linalg as npla

import threading
import time

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

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
        self.limbs = {'r_arm':self.r_arm, 'l_arm':self.l_arm}

        print("eef_link: " + self.r_arm.get_end_effector_link())
        self.eef_link = self.r_arm.get_end_effector_link()

        # setting up object and parameters needed
        self.object_name = "object"
        self.object_size = (0.1, 0.1, 0.1)

        self.slide = False
        self.roll = False
        self.pivot = False

        self.push = False

    def set_planning_params(self, limb, time = 10, attempts = 1):
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
        print("-- debug: set start")

        self.limbs[limb].set_start_state_to_current_state()

        print("-- debug: set target")

        self.limbs[limb].set_position_target([x,y,z])

        print("-- debug: plan")

        self.limbs[limb].plan()

        print("-- debug: set target")

        self.limbs[limb].go(wait=True)


        print("-- debug: stop and clear")

        self.limbs[limb].stop()
        self.limbs[limb].clear_pose_targets()

    def go_to_orientation(self, limb, roll, pitch, yaw):

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        self.limbs[limb].set_start_state_to_current_state()

        self.limbs[limb].set_orientation_target(quat)
        self.limbs[limb].go(wait=True)

        self.limbs[limb].stop()
        self.limbs[limb].clear_pose_targets()

    def get_pose(self, limb):
        return self.limbs[limb].get_current_pose()

    def get_position(self, limb):
        pose_msg = self.limbs[limb].get_current_pose()

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z

        pos = [x, y, z]
        return pos

    def get_orientation(self, limb):
        pose_msg = self.limbs[limb].get_current_pose()

        x = pose_msg.pose.orientation.x
        y = pose_msg.pose.orientation.y
        z = pose_msg.pose.orientation.z
        w = pose_msg.pose.orientation.w

        quat = [x, y, z, w]
        return quat

    def add_box(self, gripper = "right_gripper", timeout=4):
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = gripper
        object_pose.pose.orientation.w = 1.0

        self.scene.add_box(self.object_name, object_pose, size = self.object_size)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        grasping_group = "right_gripper"
        touch_links = self.robot.get_link_names(group = grasping_group)
        self.scene.attach_box(self.eef_link, self.object_name, touch_links = touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.object_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.object_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = self.scene.get_attached_objects([self.object_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = self.object_name in self.scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def check_sliding(self):
        if self.object.pos.get_curr() - self.object.pos.get_previous() > tolerance:
            self.sliding = true



'''
    def add_object(self, object_size, timeout = 4):
        self.held_object.set_size(object_size)
        self.scene.add_box(self.object_name, self.held_object, size = object_size)
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
        self.scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
'''



class HeldObject(geometry_msgs.msg.PoseStamped):

    def __init__(self, *args, **kwds):
        super(geometry_msgs.msg.PoseStamped, self).__init__(*args, **kwds)
        self.x = 0
        self.y = 0
        self.z = 0

        self.contact_points = {}

        self.sliding = False
        self.rolling = False
        self.pivoting = False

    def set_size(self, xyz):
        self.x = xyz[0]
        self.y = xyz[1]
        self.z = xyz[2]

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

# do we need this?

class Tracker(object):


    def __init__(self, tracked_object, holder):
        self.updating = False
        self.positiion = np.zeros(3,1)
        self.orientation = np.zeros(3,1)
        self.tracked_object = tracked_object
        self.holder = holder

    def track_object(self):
        while self.updating == True:
            self.position = self.get_position()
            self.orientation = self.get_orientation()
            self.update_object()

    def update_object(self):
        self.collision_check()

#    def collision_check(self):

class Point(object):

    def __init__(self,x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, point):
        return Point(self.x + point.x, self.y + point.y, self.z + point.z,)

    def __sub__(self, point):
        return Point(self.x - point.x, self.y - point.y, self.z - point.z,)


def main():


    baxter = BaxterSim()
    baxter.set_planning_params("r_arm")
    curr_pos = baxter.get_position("r_arm")
    # resting: [0.9089723296003718, -1.1039755779078562, 0.3209760000039386]
    # ...or in baxterland: [x=0.14844835898925912, y=-0.43258268259271665, z=-0.7116723052429483]
    # [0.8,-1.0,0.3]
    print("moveit curr pos:")
    print(curr_pos)

    bi_right = baxter_interface.Limb("right")
    curr_pos_bi = bi_right.endpoint_pose()
    print("baxter curr pos:")
    print(curr_pos_bi["position"])



    # baxter.go_to_position("r_hand",curr_pos[0] + 0.1, curr_pos[1] + 0.1, curr_pos[2] - 0.2)
    #baxter.add_box()
    #baxter.attach_box()
    baxter.go_to_position("r_arm", 0.2, -0.6, -0.5)

    rospy.spin()

if __name__ == '__main__':
    main()
