#! /usr/bin/env python

import subprocess
import rospy
import time
import os
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose 
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory

bagname = "WithoutPeopleEmpty_withoutodom.bag"
class Handler:
    def make_pose_covariance_stamped_msg(self, pos, quat, cov):
        """
        Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
        NOTE: Does not set the target frame.
        """
        pose_cov_stamped_msg = PoseWithCovarianceStamped()
        #
        pose_cov_stamped_msg.header = Header()
        pose_cov_stamped_msg.header.frame_id = 'robot_map'
        pose_cov_stamped_msg.header.stamp = rospy.Time.now()
        #
        pose_msg = Pose()
        pose_msg.position = pos
        #
        pose_msg.orientation = quat
        #
        pose_cov_stamped_msg.pose.pose = pose_msg
        pose_cov_stamped_msg.pose.covariance = cov
    
        return pose_cov_stamped_msg
    
    def activateSlamtoolbox(self, msg, period):
        header = msg.header
        pose = msg.pose.pose
        cov = msg.pose.covariance 
              
        # start another time
        subprocess.Popen('roslaunch slam_toolbox localization.launch bag_filename:=/root/workspace/newdisk/'+self.bagname + ' start_time:={}'.format(period), shell=True)
        while True:
           topics = rospy.get_published_topics()
           found = False
           for item in topics:
               print("got:",item)
               if item[0]=='/slam_toolbox/update':
                   found = True
           if found:
               break
        time.sleep(0.5)
        print("published:", self.make_pose_covariance_stamped_msg(pose.position, pose.orientation, cov))
        self.initialpose_pub.publish(self.make_pose_covariance_stamped_msg(pose.position, pose.orientation, cov))
        subprocess.Popen('rosbag record /clock /tf /tf_static /robot/robotnik_base_control/odom -o '+ self.bagname[:-4], shell=True)
        exit()

    def activateCartographer(self, msg, period):
        # start another time
        subprocess.Popen('roslaunch cartographer_ros demo_my_robot_localization.launch load_state_filename:=/root/workspace/newdisk/WithoutPeopleEmptyNewStart.bag.pbstream bag_filename:=/root/workspace/newdisk/'+self.bagname + ' start_time:={}'.format(period), shell=True)
        while True:
           topics = rospy.get_published_topics()
           found = False
           for item in topics:
               if item[0]=='/tracked_pose':
                   found = True
           if found:
               break
        self.finishTraj(1)
        configuration_directory = '/root/catkin_ws/install_isolated/share/cartographer_ros/configuration_files'
        configuration_basename = 'my_robot_localization.lua'
        self.startTraj(configuration_directory, configuration_basename, True, msg.pose.pose, 0)
        exit()


    def poseCallback(self, msg):
        cov = msg.pose.covariance 
        if self._enabled == False: # check for global_localization enabled or not
            if cov[0] > 5:
                self._enabled = True
                print("cov[0]:", cov[0])
        else:
            if cov[0] < 0.05: # and cov[7] < 0.05: # already converge 
                if not self._activated: # to avoid multiple trigger
                    self._activated = True
                    ret = subprocess.check_output(['rosnode list | grep -v gmcl_slamtoolbox_combine | grep -v record |  xargs rosnode kill'], shell=True) # kill all the nodes
                    time.sleep(2)
                    print("waiting to stop")
                    start_time = subprocess.check_output(['rosbag info /root/workspace/newdisk/'+self.bagname + ' | grep start | awk -F "[()]" \'{print $2}\''], shell=True)
                    period = msg.header.stamp.secs + float(msg.header.stamp.nsecs)/1e9 - float(start_time)
                    print("period:",period)

                    self.activateSlamtoolbox(msg, period)
                    #self.activateCartographer(msg, period)
    
    def __init__(self):
        self.sub = rospy.Subscriber('/gmcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        self.startTraj = rospy.ServiceProxy('start_trajectory', StartTrajectory)
        self.finishTraj = rospy.ServiceProxy('finish_trajectory', FinishTrajectory)
        self._enabled = False    
        self._activated = False
        self.bagname = bagname                                 

if __name__ == "__main__":       
    subprocess.Popen('roslaunch gmcl gmcl_omni.launch map_file:=/root/workspace/map/OGM_empty.pgm.yaml bag_filename:=/root/workspace/newdisk/'+ bagname, shell=True)
    rospy.init_node("gmcl_slamtoolbox_combine")
    handler = Handler()          
    rospy.spin()                 
                                 
                                 
                                 
                                 
                                 
                                 
