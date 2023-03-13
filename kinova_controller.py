#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   kinova_controller.py
@Time    :   2023/03/13 15:38:41
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

# here put the import lib


import copy
import json
import sys
import time

import geometry_msgs.msg
import rospy
from kinova_engine import PickAndPlace
from std_srvs.srv import Empty


class kinovaController(object):
    '''
    The main controller to the kinova arm, only implement the cube-pick-up function
    '''
    def __init__(self):
        '''
        init the main engine and load some parameters
        '''
        self.engine = PickAndPlace()
        with open('./kinova_pickupsetting.json') as f:
            self.paras = json.load(f)
            f.close()
        
        self.cubes = self.paras["environment"]["cubes"]
        self.target = self.paras["environment"]["goal"]
        self.gripper_pos = self.paras["pick_cube_para"]["angle"]
        self.gripper_height = self.paras["pick_cube_para"]["height"]
        self.finished_cube_num = 0

    def newPos(self, pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w):
        '''
        Returns a msg.pose() position from given position and orientation
        '''
        pose = geometry_msgs.msg.Pose()
        pose.position.x = pos_x
        pose.position.y = pos_y
        pose.position.z = pos_z
        
        pose.orientation.x = ori_x
        pose.orientation.y = ori_y
        pose.orientation.z = ori_z
        pose.orientation.w = ori_w    
        
        return pose

    def move_gripper(self,pose,attempts = 5):
        '''
        Move the gripper within a little distance, thank @Tim for suggestion
        '''
        tolerance = 0.01
        for i in range(attempts):
            success = self.engine.reach_cartesian_pose(pose=pose, tolerance=tolerance, constraints=None)
            if success:
                return True
        
        rospy.loginfo("The gripper failed to plan even after {} times of trying.".format(attempts))
        return False          

    def pick_up_cube(self,cube_position):
        '''
        pick up a cube at a given position
        '''
        
        x,y = cube_position["x"],cube_position["y"]
        success = True
        # move to the cube
        pose = self.newPos(pos_x=x,pos_y=y,pos_z=self.gripper_height["over"],
                           ori_x=self.gripper_pos["ori_x"],ori_y=self.gripper_pos["ori_y"],
                           ori_z=self.gripper_pos["ori_z"],ori_w=self.gripper_pos["ori_w"])
        #success &= self.engine.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
        success &= self.move_gripper(pose=pose)
        # make sure gripper is open
        success &= self.engine.reach_gripper_position(self.paras["pick_cube_para"]["gripper"]["opened"])
        # lower the gipper, may fail!
        pose.position.z = self.gripper_height["pick"]
        success &= self.move_gripper(pose=pose)
        # close the gripper
        success &= self.engine.reach_gripper_position(self.paras["pick_cube_para"]["gripper"]["closed"])
        # move up
        pose.position.z = self.gripper_height["over"] + 0.1
        success &= self.move_gripper(pose=pose)
        if success:
            print("pick up cube successfully!")
        return success
        
        
    def put_cube_to_target(self,target_position):
        '''
        Move the gripper to the target position
        '''
        x,y = target_position["x"],target_position["y"]
        success = True
        # move to the cube
        pose = self.newPos(pos_x=x,pos_y=y,pos_z=self.gripper_height["over"]+self.finished_cube_num*self.cubes["size"],
                           ori_x=self.gripper_pos["ori_x"],ori_y=self.gripper_pos["ori_y"],
                           ori_z=self.gripper_pos["ori_z"],ori_w=self.gripper_pos["ori_w"])
        #success &= self.engine.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
        success &= self.move_gripper(pose=pose)
        # move to target height
        pose.position.z = self.finished_cube_num*self.cubes["size"] + self.gripper_height["pick"]
        success &= self.move_gripper(pose=pose)
        self.finished_cube_num += 1
        #drop
        success &= self.engine.reach_gripper_position(self.paras["pick_cube_para"]["gripper"]["opened"])
        #lift
        pose.position.z = self.gripper_height["over"]+0.1
        success &= self.move_gripper(pose=pose)
        if success:
            print("place to target succesfully!")
        return success
    
    def finish(self, target_pos = 'vertical'):
        '''
        performs certain actions
        '''
        success = True
        if target_pos not in ['vertical','home','retract']:
            rospy.loginfo('target position not supported, default vertical')
            target_pos = 'vertical'
        rospy.loginfo("Finishing by reaching {} Position".format(target_pos))
    
        success &= self.engine.reach_named_position(target_pos)
        rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)
        if not success:
            rospy.logerr("Error occured when trying to reach {} Position".format(target_pos))
    
    def main(self):
        '''
        The main procedure of the arm
        '''
        success = True
        success &= self.pick_up_cube(cube_position=self.cubes["cube1"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])
        success &= self.pick_up_cube(cube_position=self.cubes["cube2"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])
        success &= self.pick_up_cube(cube_position=self.cubes["cube3"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])        
        return success        
        
    def debug(self):
        success = True
        success &= self.pick_up_cube(cube_position=self.cubes["cube1"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])
        success &= self.pick_up_cube(cube_position=self.cubes["cube2"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])
        success &= self.pick_up_cube(cube_position=self.cubes["cube3"])
        success &= self.put_cube_to_target(target_position=self.target['goal1'])        
        return success
    
if __name__ == '__main__':
    
    controller = kinovaController()
    controller.main()
    controller.finish('retract')
    
  