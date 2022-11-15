# Usage: remove robot_map -> robot_odom transformation in the bagfiles
# e.g. applications like slam_toolbox depends heavily on /tf topic
# Note: this script has to be used in company with the modified bag.py scripts.
import rosbag
senarios=["WithoutPeopleEmpty","WithPeopleEmpty","WithoutPeopleReality","WithPeopleReality","WithoutPeopleDisaster","WithPeopleDisaster"]
for senario in senarios:
    with rosbag.Bag(senario + '_withoutodom.bag', 'w') as outbag:
        inbag = rosbag.Bag(senario + '.bag')
        for topic, msg, t, conn_header in inbag.read_messages(return_connection_header=True):
            if topic == "/tf" and msg.transforms and msg.transforms[0].header.frame_id == "robot_map" and msg.transforms[0].child_frame_id == "robot_odom":
                # ignore this /tf topic
                continue
            else:
                outbag.write(topic, msg, t, connection_header=conn_header)

