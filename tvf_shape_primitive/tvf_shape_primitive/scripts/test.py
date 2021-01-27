#!/usr/bin/env python

import rosbag
from rospkg import RosPack

ROSPACK = RosPack()
PACKAGE_PATH = ROSPACK.get_path("vf_construction")
FOLDER_PATH = PACKAGE_PATH + "/meshes/hemisphere_cutout/"

bag = rosbag.Bag(FOLDER_PATH + 'hemisphere_cutout_pose_array_1.bag')
for topic, msg, t in bag.read_messages(topics=['PoseArray']):
    print msg
bag.close()
