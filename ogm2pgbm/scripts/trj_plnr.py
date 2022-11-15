#!/usr/bin/python
import os
import sys
import cv2
import time
import math
import rospy
import tf
import thread
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import MarkerArray, Marker
from skimage.morphology import skeletonize_3d
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.msg import ModelStates, ModelState
from rosgraph_msgs.msg import Clock
sys.path.append(os.path.join(os.path.dirname(__file__),'./WavefrontCPP'))
from wavefront_coverage_path_planner import WavefrontCPP 
#from WavefrontCPP.wavefront_coverage_path_planner import WavefrontCPP 

np.set_printoptions(threshold=sys.maxsize)
NAV_GOAL = False

class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.is_working = False
        self.timestamp = None
        self.state = [0, 0]

        self.br = tf.TransformBroadcaster()
        self.map_subscriber = rospy.Subscriber("/robot/map", OccupancyGrid, self.new_map_callback)
        self.state_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelstate_callback)
        self.posearr_publisher = rospy.Publisher("/pose_array", PoseArray, queue_size=1)
        self.navgoal_publisher = rospy.Publisher("/robot/move_base_simple/goal", PoseStamped, queue_size=1)
        self.state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState , queue_size=1)
        self.cloud_publisher = rospy.Publisher("/scan", LaserScan , queue_size=1)
        self.imu_publisher = rospy.Publisher("/imu",  Imu, queue_size=1)
        self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size=10)
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
        thread.start_new_thread( self.pub_clock, () )
        #self.pos_publisher = rospy.Publisher("/robot/initialpose",  PoseWithCovarianceStamped, queue_size=1)

    def pub_clock(self):
        while not rospy.is_shutdown():
            t = rospy.Time.now()
            msg = Clock()
            msg.clock = t
            self.clock_publisher.publish(msg)
            time.sleep(2);

    def skeletonize(self):
        self.info = self.map.info
        data = self.map.data
        data = np.array(data).reshape((self.info.height, self.info.width))
        data[data==0]   = 1 # (free 0)
        data[data==-1]  = 0   # (unknown -1)
        data[data==100] = 0   # (occupied 100)
        print("skeletonizing")
        self.skeleton = skeletonize_3d(data)#, method='lee') 
        self.idx = np.where(self.skeleton==1)
        self.waypoints = self.indices_to_coord(self.idx[0], self.idx[1])
        #plt.imshow(data, cmap=plt.cm.binary)
        #plt.show()
        time.sleep(1) # wait 1 seconds for the startup of rviz
        self.pub_path()

    def coord_to_indices(self, x, y):
        i = int((y - self.info.origin.position.y) / self.resolution)
        j = int((x - self.info.origin.position.x) / self.resolution)
        return (i, j)

    def indices_to_coord(self, i, j):
        y = i * self.info.resolution + self.info.origin.position.y
        x = j * self.info.resolution + self.info.origin.position.x
        return (x, y)

    def new_map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = grid_map
            rospy.loginfo("New map was set")
            print("working now!")
            self.skeletonize()
            self.CPP()
            #self.gazebo_run()
            self.raytracer()
            print("done!")
            quit()

    def modelstate_callback(self, modelstate):
        self.state = [modelstate.pose[2].position.x, modelstate.pose[2].position.y] # pose[ground_plane, office, robot]

    def CPP(self):
            start, goal = self.get_farest_pair()
            kernel = np.ones((2,2), np.uint8)
            img_dilation = cv2.dilate(self.skeleton, kernel, iterations=1)
            img_dilation = 1 - img_dilation ## invert image; be sure it consists of 0 and 1 
            planner = WavefrontCPP()
            DT = planner.transform(img_dilation, goal, transform_type='distance')
            #np.savetxt('dt_result.txt', DT) ## debug method
            self.DT_path = planner.wavefront(DT, start, goal)
            #planner.visualize_path(img_dilation, start, goal, self.DT_path, False) ## visualize

    def pub_Imu(self):
        imu = Imu()
        imu.header.stamp = self.timestamp
        imu.header.frame_id = 'robot_base_link'
        imu.orientation.x = 0
        imu.orientation.y = 0
        imu.orientation.z = 0
        imu.orientation.w = 1
        imu.linear_acceleration.x = 0.
        imu.linear_acceleration.y = 0.
        imu.linear_acceleration.z = 9.8
        self.imu_publisher.publish(imu)

    def pub_laserscan(self):
        debug_scan = LaserScan()
        debug_scan.header.stamp = self.timestamp #rospy.Time.now()
        debug_scan.header.frame_id = "robot_base_link"
        debug_scan.angle_min =  -math.pi / 2.0
        debug_scan.angle_increment = self.yawreso / 180 * math.pi
        # slam_toolbox uses [a,b)
        debug_scan.angle_max =  3.0/2.0 * math.pi - debug_scan.angle_increment
        debug_scan.angle_increment = self.yawreso / 180 * math.pi
        debug_scan.range_max = 10.0
        size = len(self.waypoints[0])
        def norm(x,y):
            return math.sqrt(x*x + y*y)
        debug_scan.ranges = [norm(self.waypoints[0][i]-self.origins[0], self.waypoints[1][i]-self.origins[1]) for i in range(size)]  # calc norm
        self.cloud_publisher.publish(debug_scan)

    def raytracer(self):
        data = self.map.data
        data = np.array(data).reshape((self.info.height, self.info.width))
        def check_border(x, y):
            return not (x >= 0 and x < self.info.height and y >=0 and y < self.info.width)
        def check_collision(x, y):
            return data[int(x)][int(y)] == 100 # collision detect
        def deg2rad(val):
            return val * 3.1415926 / 180
        def setposition(x, y):
            r = R.from_euler('z', 0).as_quat()
            #modelstate = ModelState()
            #modelstate.model_name = "robot"
            #modelstate.pose.position.x = x
            #modelstate.pose.position.y = y
            #modelstate.pose.orientation.x = r[0]
            #modelstate.pose.orientation.y = r[1]
            #modelstate.pose.orientation.z = r[2]
            #modelstate.pose.orientation.w = r[3]
            #self.state_publisher.publish(modelstate)
            #-----------------
            '''
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "robot_map"
            pose.pose.pose.position.x=x
            pose.pose.pose.position.y=y
            pose.pose.pose.position.z=0
            pose.pose.covariance=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pose.pose.pose.orientation.z=r[2]
            pose.pose.pose.orientation.w=r[3]
            '''
            #self.pos_publisher.publish(pose)
            odom = Odometry() 
            odom.header.stamp = self.timestamp
            odom.header.frame_id = "robot_odom"
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
            odom.child_frame_id = "robot_base_link"
            odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.odom_publisher.publish(odom)

            self.br.sendTransform((0, 0, 0),
                odom_quat,
                self.timestamp,
                "robot_odom",
                "robot_map")
            self.br.sendTransform((x, y, 0),
                odom_quat,
                self.timestamp,
                "robot_base_link",
                "robot_map")
            
        numBeams = 360 
        self.yawreso = 360.0/numBeams # 1 deg
        filter_num = 0
        for origin in self.DT_path:
            filter_num = filter_num + 1
            # do not use filter now, as we need more beams to cover the environment
            #if filter_num % 1 != 0:
            #    continue
            center_X, center_Y = origin[0], origin[1]
            self.origins = self.indices_to_coord(center_X, center_Y)
            # use the same timestamp for all msgs type
            self.timestamp = rospy.Time.now()
            # Imu is useless for 2d mapping
            #self.pub_Imu()
            # publish tf and odometry (important!)
            setposition(self.origins[0], self.origins[1])
            raycast = [[] for i in range(numBeams)]
            # generate lidar data in the following for loop
            for num in range(numBeams):
                theta_rad = deg2rad(num*self.yawreso)
                '''
                pixel frame  |  global map frame |   
                        ^ y  |        ^ x        |
                        |    |        |          |
                  x <----    |  y <----          |
                '''
                dir = [math.sin(theta_rad), math.cos(theta_rad)]
                tangent = dir[0] / dir[1] if dir[1] != 0 else 99
                index = 1
                while 1:
                    xsign =  1 if dir[1] > 0 else -1
                    ysign =  1 if dir[0] > 0 else -1
                    if tangent > 1 or tangent < -1: # tan(88deg)=28.6
                        xCoord = -(1/tangent * index * ysign) + center_X
                        yCoord = ysign * index + center_Y
                    else :
                        yCoord = (tangent * index * xsign) + center_Y
                        xCoord = -xsign * index + center_X
                    if check_border(xCoord, yCoord):
                        raycast[num] = [999999, 999999]
                        break
                    if check_collision(xCoord, yCoord) :
                        raycast[num] = [xCoord, yCoord]
                        break
                    else:
                        index = index + 1
            self.rayd = np.array(raycast).T
            self.waypoints = self.indices_to_coord(self.rayd[0], self.rayd[1])
            self.pub_laserscan()
            # disable marker publisher for speed up
            #self.pub_path()

    def gazebo_run(self):
        past = None
        filter_num = 0
        for imgIdx in self.DT_path:
            if filter_num%20 != 0:
                filter_num = filter_num + 1
                continue
            waypoint = self.indices_to_coord(imgIdx[0], imgIdx[1])
            if past is not None:
                yaw = math.atan2(waypoint[1]-past[1], waypoint[0]-past[0])
                print('yaw:',yaw*180/3.1415926)
                r = R.from_euler('z', yaw, degrees=False).as_quat()
                '''
                # should not use rosservice call or rostopic pub but pub them with API directly
                # cmd = f'rosservice call /gazebo/set_model_state "{{model_state: {{ model_name: robot, pose: {{ position: {{ x: {waypoint[0]}, y: {waypoint[1]} ,z: 0 }}, orientation: {{x: {r[0]}, y: {r[1]}, z: {r[2]}, w: {r[3]} }} }}, twist: {{ linear: {{x: 0.0 , y: 0 ,z: 0 }} , angular: {{ x: 0.0 , y: 0 , z: 0.0 }} }} , reference_frame: world }} }}"'
                # for rostopic pub, use Once mode, otherwise, it may not determinate
                cmd = f'rostopic pub /robot/move_base_simple/goal geometry_msgs/PoseStamped "{{header: {{stamp: {rospy.Time.now()}, frame_id: "robot_map"}}, pose: {{position: {{x: {waypoint[0]}, y: {waypoint[1]}, z: 0.0}}, orientation: {{x: {r[0]}, y: {r[1]}, z: {r[2]}, w: {r[3]} }} }} }}"'
                print(cmd)
                res = os.system(cmd) # os.popen is non-blocking and the ret is a file, has to be closed! use os.system for simplicity
                '''
                if NAV_GOAL == True:
                    nav_goal = self.to_pose(waypoint[0], waypoint[1], r, True)
                    past = waypoint
                    self.navgoal_publisher.publish(nav_goal) 
                    while np.linalg.norm(np.array(self.state) - np.array(waypoint)) > 0.7: 
                        print("distance:", np.linalg.norm(np.array(self.state) - np.array(waypoint)))
                        time.sleep(0.2)
                else: # set state directly
                    modelstate = ModelState()
                    modelstate.model_name = "robot"
                    modelstate.pose.position.x = waypoint[0]
                    modelstate.pose.position.y = waypoint[1]
                    modelstate.pose.orientation.x = r[0]
                    modelstate.pose.orientation.y = r[1]
                    modelstate.pose.orientation.z = r[2]
                    modelstate.pose.orientation.w = r[3]

                    self.state_publisher.publish(modelstate) 
                    time.sleep(2)
            past = waypoint
            filter_num = filter_num + 1

    def pub_path(self):
        pose_arr = PoseArray()
        pose_id = 0
        idx_x, idx_y = self.waypoints
        num = len(idx_x)
        while pose_id < num:
            new_pose = self.to_pose(idx_x[pose_id], idx_y[pose_id], [0,0,0,0])
            pose_id += 1
            pose_arr.poses.append(new_pose)
        pose_arr.header.stamp = rospy.Time.now()
        pose_arr.header.frame_id = "robot_map"
        self.posearr_publisher.publish(pose_arr)
    
    def to_pose(self, x, y, quaternion, stamped=False):
        if stamped == True:
            posestamped = PoseStamped()
            posestamped.header.frame_id = "robot_map"
            posestamped.header.stamp = rospy.Time.now()

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.25

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        if stamped == True:
            posestamped.pose = pose
            return posestamped
        else:
            return pose

    def get_farest_pair(self):
        from scipy import spatial
        # two points which are fruthest apart will occur as vertices of the convex hull
        self.idx = np.vstack(self.idx).T
        candidates = self.idx[spatial.ConvexHull(self.idx).vertices]
        # get distances between each pair of candidate points
        dist_mat = spatial.distance_matrix(candidates,candidates)
        # get indices of candidates that are furthest apart
        i,j = np.unravel_index(dist_mat.argmax(),dist_mat.shape)
        return (tuple(candidates[i]),tuple(candidates[j]))


def main():
    rospy.init_node("trajectory_planner")
    planner = TrajectoryPlanner()
    rospy.spin()

main()
