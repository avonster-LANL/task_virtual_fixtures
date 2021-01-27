#!/usr/bin/env python
"""Classes for loading frames and collision meshes based on known AR tags
detected by RGBD camera."""

from os import listdir
from math import isnan
from numpy import empty, mean
from rospkg import RosPack
from rospy import init_node, is_shutdown, signal_shutdown, Duration, Time, \
    sleep, get_param, Subscriber
from geometry_msgs.msg import Pose, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf import transformations as tfs, TransformListener, TransformBroadcaster, \
    LookupException, ConnectivityException, ExtrapolationException, \
    Exception as TfE
from support_utilities.MessagesExits import MessagesExits as Me
from support_utilities.ManipulateMeshes import ManipulateMeshes

__author__ = "Andrew Sharp"
__maintainer__ = "Andrew Sharp"
__email__ = "asharp@utexas.edu"
__credits__ = "Andrew Sharp"
__license__ = "BSD"
__copyright__ = """Copyright The University of Texas at Austin, 2014-20XX.
                All rights reserved. This software and documentation
                constitute an unpublished work and contain valuable trade
                secrets and proprietary information belonging to the
                University. None of the foregoing material may be copied or
                duplicated or disclosed without the express, written
                permission of the University. THE UNIVERSITY EXPRESSLY
                DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND
                DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY
                AND/OR FITNESS FOR A PARTICULAR PURPOSE, AND WARRANTIES OF
                PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM
                COURSE OF DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER
                EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF THE SOFTWARE OR
                DOCUMENTATION. Under no circumstances shall the University be
                liable for incidental, special, indirect, direct or
                consequential damages or loss of profits, interruption of
                business, or related expenses which may arise from use of
                software or documentation, including but not limited to those
                resulting from defects in software and/or documentation, or
                loss or inaccuracy of data of any kind."""
__project__ = "Virtual Fixtures"
__version__ = "1.0.1"
__status__ = "Production"
__description__ = """Classes for loading frames and collision meshes based on
                  known AR tags detected by RGBD camera."""


class SurfaceInspectionLoader:
    """Class for reading in information from surface and inspection launch
    files."""
    def __init__(self, params):
        """Load ROS params and build dictionaries from list of strings."""
        self.params = params
        self.strs = ['files', 'surface_frames', 'surfaces', 'inspections',
                     'camera', 'rad', 'other']
        self.surface_dict = {self.strs[0]: [],
                             self.strs[1]: [],
                             self.strs[2]: {}}
        self.inspection_dict = {self.strs[0]: [],
                                self.strs[4]: {},
                                self.strs[5]: {},
                                self.strs[6]: {}}
        ros_pack = RosPack()
        self.package_path = ros_pack.get_path(self.params['package'])
        return

    def fill_surface_dict(self, folder):
        """Fill dictionary with surface information from launch files."""
        for file_name in listdir(self.package_path + folder):
            if 'ar_' in file_name and file_name.endswith(".launch"):
                self.surface_dict[self.strs[0]].append(file_name)
                surface = file_name.split('.')[0]
                params = get_param(surface)
                if len(self.surface_dict[self.strs[2]].keys()) == 0:
                    self.surface_dict[self.strs[2]] = {surface: params}
                else:
                    self.surface_dict[self.strs[2]][surface] = params
        return self.surface_dict

    def fill_inspection_dict(self, folder):
        """Fill dictionary with inspection information from launch files."""
        for file_name in listdir(self.package_path + folder):
            if 'ar_' in file_name and file_name.endswith(".launch"):
                self.inspection_dict[self.strs[0]].append(file_name)
                inspection = file_name.split('.')[0]
                params = get_param(inspection)
                for key in self.inspection_dict:
                    if key in inspection and key not in self.strs[0]:
                        if len(self.inspection_dict[key].keys()) == 0:
                            self.inspection_dict[key] = \
                                {inspection: params}
                        elif len(self.inspection_dict[key].keys()) != 0:
                            self.inspection_dict[key][inspection] = params
                        else:
                            Me.error_message('Problem with size of '
                                             'inspection dictionary.')
                    else:
                        # don't do anything & continue if key is for file names
                        continue

        return self.inspection_dict


class ARFrameLoader:
    """Class for adding frames at detected AR tag locations."""
    def __init__(self, num_to_avg):
        """Read ROS parameters, set up tf classes and initialize dictionary for
        ar_barcode storage."""
        self.params = {'debug': get_param('~debug'),
                       'world': get_param('~world_frame'),
                       'package': get_param('~package')}
        self.num_to_avg = num_to_avg
        self.tbr = TransformBroadcaster()
        self.tfl = TransformListener()
        self.ar_barcodes = {}
        return

    def ar_callback(self, data):
        """Build a list of the AR tags detected."""
        marker_list = data.markers
        for item in marker_list:
            if not isnan(item.pose.pose.position.x):
                ar_marker = "/ar_marker_" + str(item.id)
                ar_frame = 'ar_' + str(item.id)
                xyz_avg, xyzw_avg = \
                    self.get_average_pose(num_to_avg=self.num_to_avg,
                                          ar_marker=ar_marker)
                if xyz_avg != (0.0, 0.0, 0.0):
                    # If marker is already present just update location
                    if ar_frame in self.ar_barcodes.keys():
                        self.ar_barcodes[ar_frame] = [xyz_avg, xyzw_avg]
                    # Add marker if not already present
                    else:
                        # If dictionary is empty add first entry
                        if len(self.ar_barcodes.keys()) == 0:
                            self.ar_barcodes = {ar_frame: [xyz_avg, xyzw_avg]}
                        # If dictionary has entries add a new one
                        else:
                            self.ar_barcodes[ar_frame] = [xyz_avg, xyzw_avg]
                else:
                    # Transform data too old
                    continue
            else:
                # False AR tag detection
                continue
        # sleep(1.0)
        return self.ar_barcodes

    def get_average_pose(self, num_to_avg, ar_marker):
        """Get the specified number of poses and average them."""
        x_positions, y_positions, z_positions, roll_orientations, \
            pitch_orientations, yaw_orientations = \
            empty([num_to_avg]), empty([num_to_avg]), \
            empty([num_to_avg]), empty([num_to_avg]), \
            empty([num_to_avg]), empty([num_to_avg])

        for i in range(num_to_avg):
            # Fill in pose stamped and check for transform
            pose = PoseStamped()
            pose.header.frame_id = ar_marker
            pose.pose.orientation.w = 1.0
            try:
                self.tfl.waitForTransform(self.params['world'], ar_marker,
                                          Time(0), Duration(2))
                pose = self.tfl.transformPose(self.params['world'], pose)
            except TfE:
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            # switch to Euler angles
            euler_angles = \
                tfs.euler_from_quaternion(
                    [pose.pose.orientation.x,
                     pose.pose.orientation.y,
                     pose.pose.orientation.z,
                     pose.pose.orientation.w])
            # Record values to be averaged
            x_positions[i] = pose.pose.position.x
            y_positions[i] = pose.pose.position.y
            z_positions[i] = pose.pose.position.z
            roll_orientations[i] = euler_angles[0]
            pitch_orientations[i] = euler_angles[1]
            yaw_orientations[i] = euler_angles[2]
            # Sleep to ensure averaging of different transforms
            sleep(0.02)

        # Average the collected data
        xyz_avg = (mean(x_positions), mean(y_positions),
                   mean(z_positions))
        quaternion_avg = tfs.quaternion_from_euler(mean(roll_orientations),
                                                   mean(pitch_orientations),
                                                   mean(yaw_orientations))
        xyzw_avg = (quaternion_avg[0], quaternion_avg[1],
                    quaternion_avg[2], quaternion_avg[3])
        return xyz_avg, xyzw_avg

    def load_ar_frames(self):
        """Use tf to place frames at filtered locations."""
        if len(self.ar_barcodes.keys()) != 0:
            for ar_barcode in self.ar_barcodes.keys():
                try:
                    xyz_avg = self.ar_barcodes[ar_barcode][0]
                    xyzw_avg = self.ar_barcodes[ar_barcode][1]
                    self.tbr.sendTransform(xyz_avg, xyzw_avg, Time.now(),
                                           ar_barcode, self.params['world'])
                    # Check frame exists
                    self.tfl.waitForTransform(self.params['world'], ar_barcode,
                                              Time(0), Duration(2))
                    if self.params['debug']:
                        Me.info_message(ar_barcode + " verified")

                except (TfE, LookupException, ConnectivityException,
                        ExtrapolationException):
                    Me.error_message("Error loading " + ar_barcode + " frame.")
        return


class SurfaceFrameLoader:
    """Class for adding frames for detected surfaces based on detected AR tag
    number."""
    def __init__(self):
        """Read ROS parameters, set up tf classes, initialize dictionary for
        ar_barcode storage, and read surface information from launch files."""
        self.params = {'debug': get_param('~debug'),
                       'world': get_param('~world_frame'),
                       'package': get_param('~package')}
        self.tbr = TransformBroadcaster()
        self.tfl = TransformListener()
        self.ar_frames = {}

        self.strs = ['files', 'surfaces', 'inspections', 'camera', 'rad',
                     'other', 'frame_translation', 'frame_rotation']
        sil = SurfaceInspectionLoader(params=self.params)
        self.surface_dict = sil.fill_surface_dict(folder='/surfaces/')
        return

    def get_ar_frames(self):
        """Build a list of the AR frames detected."""
        for i in range(0, 244, 1):
            frame = 'ar_' + str(i)
            try:
                if self.tfl.lookupTransform(frame, self.params['world'],
                                            Time(0)) and \
                        frame not in self.surface_dict[self.strs[1]]:
                    pose = PoseStamped()
                    pose.header.frame_id = frame
                    pose.header.stamp = Time.now()
                    # If marker is already present just update location
                    if frame in self.ar_frames.keys():
                        self.ar_frames[pose.header.frame_id] = pose
                    # Add marker if not already present
                    else:
                        # If dictionary is empty add first entry
                        if len(self.ar_frames.keys()) == 0:
                            self.ar_frames = {pose.header.frame_id: pose}
                        # If dictionary has entries add a new one
                        else:
                            self.ar_frames[pose.header.frame_id] = pose
                else:
                    # Surface frame already in list
                    continue
            except TfE:
                continue
            except KeyboardInterrupt:
                break
        return self.ar_frames

    def load_surface_frames(self):
        """Load surface frames based on tag and loaded information."""
        Me.info_message("AR frames: " + self.ar_frames)
        if len(self.ar_frames.keys()) != 0:
            for ar_frame in self.ar_frames.keys():
                surface_frame = ar_frame + '_surface'
                # Check surface frames
                if surface_frame in self.surface_dict[self.strs[1]].keys():
                    surface = self.surface_dict[self.strs[1]][surface_frame]
                    surfaces = len(surface[self.strs[6]])
                    for i in range(surfaces):
                        if surfaces > 1.0:
                            frame_name = surface_frame + '-' + str(i)
                        else:
                            frame_name = surface_frame
                        # Get transform values from launch file
                        xyz = surface[self.strs[6]][i]
                        xyzw = tfs.quaternion_from_euler(
                            surface[self.strs[7]][i][0],
                            surface[self.strs[7]][i][1],
                            surface[self.strs[7]][i][2])
                        # Build into pose for transform to base frame
                        ar_pose = PoseStamped()
                        ar_pose.header.frame_id = ar_frame
                        ar_pose.header.stamp = \
                            self.ar_frames[ar_frame].header.stamp
                        ar_pose.pose.position.x = xyz[0]
                        ar_pose.pose.position.y = xyz[1]
                        ar_pose.pose.position.z = xyz[2]
                        ar_pose.pose.orientation.x = xyzw[0]
                        ar_pose.pose.orientation.y = xyzw[1]
                        ar_pose.pose.orientation.z = xyzw[2]
                        ar_pose.pose.orientation.w = xyzw[3]
                        try:
                            # Convert pose to base frame
                            ar_pose.header.stamp = self.tfl. \
                                getLatestCommonTime(self.params['world'],
                                                    ar_pose.header.frame_id)
                            base_pose = \
                                self.tfl.transformPose(self.params['world'],
                                                       ar_pose)
                        except (TfE, LookupException, ConnectivityException):
                            Me.error_message("Problem transforming pose " +
                                             frame_name)
                            continue
                        # Change back to list to publish transform
                        xyz = [base_pose.pose.position.x,
                               base_pose.pose.position.y,
                               base_pose.pose.position.z]
                        xyzw = [base_pose.pose.orientation.x,
                                base_pose.pose.orientation.y,
                                base_pose.pose.orientation.z,
                                base_pose.pose.orientation.w]
                        try:
                            # Publish transform
                            self.tbr.sendTransform(xyz, xyzw, Time.now(),
                                                   frame_name,
                                                   self.params['world'])
                            # Check frame exists
                            self.tfl.waitForTransform(self.params['world'],
                                                      frame_name,
                                                      Time(0), Duration(2))
                            if self.params['debug']:
                                Me.info_message(frame_name + " verified")
                        except (TfE, LookupException, ConnectivityException):
                            Me.error_message("Error loading " + frame_name)
                            continue
        return


class SurfaceCollisionLoader:
    """Class for adding collision models for detected surfaces based on detected
    AR tag number."""
    def __init__(self):
        """Read ROS parameters, set up tf classes, initialize dictionary for
        ar_barcode storage, read surface information from launch files, and
        initialize mesh loader class."""
        self.params = {'debug': get_param('~debug'),
                       'world': get_param('~world_frame'),
                       'package': get_param('~package')}
        self.tbr = TransformBroadcaster()
        self.tfl = TransformListener()
        self.strs = ['files', 'surface_frames', 'surfaces',
                     'mesh_translation', 'mesh_rotation']
        sil = SurfaceInspectionLoader(params=self.params)
        self.surface_dict = sil.fill_surface_dict(folder='/surfaces/')

        mesh_folder_path = '/meshes/'
        self.meshes = ManipulateMeshes(package=self.params['package'],
                                       folder_path=mesh_folder_path)
        return

    def surface_frames(self):
        """Build a list of the AR surface frames detected."""
        for i in range(0, 244, 1):
            frame = 'ar_' + str(i) + '_surface'
            try:
                if self.tfl.lookupTransform(frame, self.params['world'],
                                            Time(0)) and \
                        frame not in self.surface_dict[self.strs[1]]:
                    self.surface_dict[self.strs[1]].append(frame)
                else:
                    # Surface frame already in list
                    continue
            except TfE:
                continue
            except KeyboardInterrupt:
                break
        return self.surface_dict[self.strs[1]]

    def load_surface_collision(self):
        """Load task surface frame based on tag and loaded information."""
        if len(self.surface_dict[self.strs[1]]) != 0:
            for surface_frame in self.surface_dict[self.strs[1]]:
                surface_frame = surface_frame.split('-')[0]
                # Check surface frames
                if surface_frame in self.surface_dict[self.strs[2]].keys():
                    surface = self.surface_dict[self.strs[2]][surface_frame]
                    surfaces = len(surface['mesh_translation'])
                    for i in range(surfaces):
                        if i == 0:
                            frame_name = surface_frame
                            object_id = surface['name']
                        else:
                            frame_name = surface_frame + '-' + str(i)
                            object_id = surface['name'] + '_' + str(i)
                        # Get transform values from launch file
                        xyz = surface[self.strs[3]][i]
                        xyzw = tfs.quaternion_from_euler(
                            surface[self.strs[4]][i][0],
                            surface[self.strs[4]][i][1],
                            surface[self.strs[4]][i][2])
                        # Build into pose
                        pose = Pose()
                        pose.position.x = xyz[0]
                        pose.position.y = xyz[1]
                        pose.position.z = xyz[2]
                        pose.orientation.x = xyzw[0]
                        pose.orientation.y = xyzw[1]
                        pose.orientation.z = xyzw[2]
                        pose.orientation.w = xyzw[3]
                        # Load mesh
                        self.meshes.load(object_id=object_id,
                                         filename=surface['file'][i],
                                         pose_in=pose, frame=frame_name)


def main():
    """Start AR frame, surface frame, and collision mesh loader in one node."""
    # Initialize node
    name = 'AR_scene_loader'
    Me.info_message('Starting ' + name + ' node.')
    init_node(name)
    Me.info_message(name + ' node initialized.')

    # Detect scene objects with known markers and average poses
    arf = ARFrameLoader(num_to_avg=100)
    Subscriber('ar_pose_marker', AlvarMarkers, arf.ar_callback)
    Me.info_message('AR_frame_loader subscriber initialized.')

    # Detect tf frames from known markers
    suf = SurfaceFrameLoader()
    # Load collision models at tagged surface frames
    scl = SurfaceCollisionLoader()

    Me.info_message(name + ' starting loop.')
    # Loop functions
    while not is_shutdown():
        arf.load_ar_frames()

        suf.get_ar_frames()
        suf.load_surface_frames()

        frames = scl.surface_frames()
        # Me.info_message('Surface frames detected: ' + str(frames))
        scl.load_surface_collision()

        sleep(0.5)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()

if __name__ == "__main__":
    main()
