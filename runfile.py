#!/usr/bin/env python

import hampus_planner_2

import baxter_interface

right = baxter_interface.Limb("right")
left = baxter_interface.Limb("left")
planner = hampus_planner_2.BaxterSim()

def main():
	try:
    	while True:
        	print("right current eef-pose: " + right.endpoint_pose())
        	move = raw_input("delta_pos: ")
        	print("planning and going")
        	planner.set_planning_params("r_arm")
        	

	except KeyboardInterrupt:
    	print('interrupt')

if __name__ == '__main__':
    main()
