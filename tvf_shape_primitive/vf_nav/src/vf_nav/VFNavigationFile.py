#!/usr/bin/env python
"""This class is designed to provide step teleoperation based on a VF retrieved
 from the SurfaceVFServer."""

from sys import path
from copy import deepcopy
from math import sqrt
from rospkg import RosPack
from rospy import init_node, Duration, Time, Publisher, sleep
from tf import TransformListener, Exception as TfE, LookupException, \
    ConnectivityException
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, PoseArray
from view_controller_msgs.msg import CameraPlacement
from vf_construction.msg import SurfaceVFAction, SurfaceVFGoal

'''ROSPACK = RosPack()
PACKAGE_PATH = ROSPACK.get_path("support_utilities")
FOLDER_PATH = PACKAGE_PATH + "/scripts"
print(path)
if FOLDER_PATH not in path:
    PATH = path.insert(0, FOLDER_PATH)
print(path)'''
from support_utilities.ArmParametersFile import ArmParameters
from support_utilities.ArmFunctionsFile import ArmFunctions
from support_utilities.MessagesExitsFile import MessagesExits as Me
from support_utilities.ManipulateMarkersFile import ManipulateMarkers

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
__description__ = """This class provides step teleop operation by passing poses
                  to MoveIt."""


class VFNavigation:
    """Provides step teleop operation through descartes by passing poses to
    an action server."""
    def __init__(self, params):
        """Initialize the action client and create a task object to receive
        and inspection matrix"""
        self.params = params

        # Initialize marker classes
        self.markers = ManipulateMarkers()
        self.pub = Publisher("/rviz/camera_placement", CameraPlacement,
                             queue_size=1)
        self.tfl = TransformListener()
        self.clients = \
            {'polar_camera': SimpleActionClient("VF_Polar_camera_server",
                                                SurfaceVFAction),
             'polar_rad': SimpleActionClient("VF_Polar_rad_server",
                                             SurfaceVFAction),
             'cartesian_camera':
                 SimpleActionClient("VF_Cartesian_camera_server",
                                    SurfaceVFAction),
             'cartesian_rad':
                 SimpleActionClient("VF_Cartesian_rad_server",
                                    SurfaceVFAction),
             'CAD_camera':
                 SimpleActionClient("VF_CAD_camera_server",
                                    SurfaceVFAction),
             'client': ''}

        Me.info_message("Waiting for VF servers.")
        for key in self.clients:
            if key != 'client':
                self.clients[key].wait_for_server(Duration(1))

        # Build empty dictionaries for use after operator input
        self.inspection = {'name': '',
                           'pose_list': [],
                           'x_size': 1,
                           'z_size': 1,
                           'y_array': []}

        self.data = {'x': {'x-': -1, 'x+': 1, 'current': 0, 'limit': 0},
                     'y': {'y-': -1, 'y+': 1, 'current': 0, 'limit': 0},
                     'z': {'z-': -1, 'z+': 1, 'current': 0, 'limit': 0},
                     'poses': {'robot': PoseStamped(),
                               'marker': PoseStamped(),
                               'path': PoseArray()}}

        # Initialize class for the arm
        self.manip = ArmParameters(group_name=self.params['group'],
                                   joint_control=True)
        # Initialize class to interface with the arms
        self.manip_fcns = \
            ArmFunctions(world_frame=self.params['world'],
                         run_rate=60, deltas={'joint': 0.01, 'teleop': 0.005},
                         fcn_params=self.params)
        return

    def define_inspection(self, client, inspection_name):
        """Get the client and inspection names from user."""
        self.inspection['name'] = inspection_name
        self.clients['client'] = client
        found = False
        while not found:
            found = \
                self.get_inspection(client=self.clients[self.clients['client']],
                                    inspection=self.inspection['name'])
        # Get the closest inspection grid location to the current EEf
        # location for nominal pose
        current_pose = \
            self.manip.group.get_current_pose(self.manip.get_group().
                                              get_end_effector_link())
        min_dist = 10e10  # Just a really big number
        min_location = [0, 0]
        for i in range((len(self.inspection['pose_list']))):
            for j in range(len(self.inspection['pose_list'][i].poses)):
                pose = PoseStamped()
                pose.header = self.inspection['pose_list'][i].header
                pose.pose = self.inspection['pose_list'][i].poses[j]
                try:
                    # Convert pose to base frame
                    pose.header.stamp = self.tfl. \
                        getLatestCommonTime(self.params['world'],
                                            pose.header.frame_id)
                    base_pose = \
                        self.tfl.transformPose(self.params['world'], pose)
                except (TfE, LookupException, ConnectivityException):
                    Me.error_message("Problem transforming pose " +
                                     pose.header.frame_id)
                    continue
                distance = sqrt((current_pose.pose.position.x -
                                 base_pose.pose.position.x) ** 2 +
                                (current_pose.pose.position.y -
                                 base_pose.pose.position.y) ** 2 +
                                (current_pose.pose.position.z -
                                 base_pose.pose.position.z) ** 2)
                if distance <= min_dist:
                    min_dist = distance
                    min_location = [i, j]
                else:
                    continue
        # Get the size of the x now array for nominal position
        y_size = self.inspection['y_array'][min_location[0]]
        z_now = int(min_location[1] / y_size)
        y_now = min_location[1] - z_now * y_size

        # Me.info_message('Minimum distance: ' + str(min_dist) + '\n' +
        #                 'x: ' + str(min_location[0]) + '\n' +
        #                 'j value: ' + str(min_location[1]) + '\n' +
        #                 'poses_size: ' +
        #                 str(len(self.inspection['pose_list']
        #                         [min_location[0]].poses)) + '\n' +
        #                 'y_size: ' + str(y_size) + '\n' +
        #                 'y_now: ' + str(y_now) + '\n' +
        #                 'z_now: ' + str(z_now))

        # Set now positions to the center of the inspection matrix
        self.data = {'x': {'x-': -1, 'x+': 1,
                           'current': min_location[0],
                           'limit': self.inspection['x_size'] - 1},
                     'y': {'y-': -1, 'y+': 1,
                           'current': y_now,
                           'limit': y_size - 1},
                     'z': {'z-': -1, 'z+': 1,
                           'current': z_now,
                           'limit': self.inspection['z_size'] - 1},
                     'poses': {'robot': PoseStamped(),
                               'marker': PoseStamped(),
                               'path': PoseArray()}}

        # Update class variables
        self.update_current_pose(string='robot')
        self.update_current_pose(string='marker')
        self.update_markers()

        Me.info_message("VF Teleoperation Initialized")
        if self.manip_fcns.pose_move(robot=self.manip.get_robot(),
                                     group=self.manip.get_group(),
                                     frames=[self.params['world']],
                                     input_poses=[self.data['poses']['robot']],
                                     links=[self.manip.get_group().
                                            get_end_effector_link()]):
            self.adjust_camera()
        return

    def vf_nav(self, command):
        """Uses the provided command (x-, x+, y-, y+, z-, z+) to choose a
        desired pose from an inspection matrix and then passes it to an action
        server."""
        # Me.info_message("Current pose:\n" + str(self.current_pose.position))
        success = False
        try:
            # Pull new VF in case the location changed
            found = False
            while not found:
                found = \
                    self.get_inspection(client=self.clients[self.clients
                                                            ['client']],
                                        inspection=self.inspection['name'])
            self.update_current_pose(string='robot')
            self.update_current_pose(string='marker')

            if 'pose' in command:
                Me.info_message("Pose command: " + command + " received.")
                success = self.pose_nav(command=command.split(':')[1])
            elif 'path' in command:
                Me.info_message("Path command: " + command + " received.")
                success = self.path_nav(command=command.split(':')[1])
            else:
                Me.error_message("Unknown pose or path command received.")
        except IndexError:
            success = False
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        return success

    def pose_nav(self, command):
        """Uses the provided command (x-, x+, y-, y+, z-, z+) to choose a
        desired pose an array and then passes it to an action server to
        execute the move."""
        found = False
        success = False
        for key in self.data:
            if command in self.data[key]:
                # Me.info_message("Command " + command + " received.")
                found = True
                self.move_marker(key=key, command=command)
                if self.manip_fcns.pose_move(robot=self.manip.get_robot(),
                                             group=self.manip.get_group(),
                                             frames=[self.params['world']],
                                             input_poses=[
                                                 self.data['poses']['marker']],
                                             links=
                                             [self.manip.get_group().
                                              get_end_effector_link()]):
                    sleep(2)
                    self.adjust_camera()
                    success = True

        if not found:
            Me.error_message("Unknown pose command received.")

        return success

    def path_nav(self, command):
        """Uses the provided command (x-, x+, y-, y+, z-, z+) to choose a
        desired pose in an array. Builds a path of points for cartesian
        execution."""
        found = False
        success = False
        if command == 'stop':
            Me.error_message("Stop command received.")
            raise KeyboardInterrupt

        elif command == 'add':
            # Me.info_message("Command " + command + " received.")
            found = True
            success = self.add_fcn(success=success)

        elif command == 'del':
            # Me.info_message("Command " + command + " received.")
            found = True
            success = self.del_fcn(success=success)

        elif command == 'plan':
            # Me.info_message("Command " + command + " received.")
            found = True
            success = \
                self.manip_fcns.plan_cart_move(robot=self.manip.get_robot(),
                                               group=self.manip.get_group(),
                                               pose_array=
                                               self.data['poses']['path'])
        elif command == 'execute':
            # Me.info_message("Command " + command + " received.")
            found = True
            success = \
                self.manip_fcns.cart_move(robot=self.manip.get_robot(),
                                          group=self.manip.get_group(),
                                          pose_array=
                                          self.data['poses']['path'])
            # Reset pose array
            self.data['poses']['path'] = PoseArray()
        elif command == 'clear':
            # Me.info_message("Command " + command + " received.")
            found = True
            # Reset pose array
            self.data['poses']['path'] = PoseArray()
        else:
            for key in self.data:
                if command in self.data[key]:
                    # Me.info_message("Command " + command + " received.")
                    found = True
                    success = self.move_marker(key=key, command=command)

        if not found:
            Me.error_message("Unknown path command received.")

        return success

    def add_fcn(self, success):
        """Check to be sure this pose isn't already in the array."""
        if len(self.data['poses']['path'].poses) == 0:
            self.add_pose()
            success = True
        elif self.data['poses']['path'].poses[-1] != \
                self.data['poses']['marker'].pose:
            self.add_pose()
            success = True
            Me.info_message("Current marker pose added to array.")
        else:
            Me.info_message("Current marker pose is the last pose in "
                            "the array.")
        return success

    def del_fcn(self, success):
        """Delete the current pose if the last pose in the path array."""
        tol_val = 0.0001
        try:
            # Convert pose to base frame
            self.data['poses']['marker'].header.stamp = \
                self.tfl.getLatestCommonTime(self.params['world'],
                                             self.data['poses']
                                             ['marker'].header.frame_id)
            marker = self.tfl.transformPose(self.params['world'],
                                            self.data['poses']['marker'])
        except (TfE, LookupException, ConnectivityException):
            Me.error_message("Error transforming pose " +
                             self.data['poses']['marker'].header.frame_id)

        dist_xyz = ((self.data['poses']['path'].poses[-1].position.x -
                     marker.pose.position.x) ** 2.0 +
                    (self.data['poses']['path'].poses[-1].position.y -
                     marker.pose.position.y) ** 2.0 +
                    (self.data['poses']['path'].poses[-1].position.z -
                     marker.pose.position.z) ** 2.0) ** 0.5
        dist_xyzw = \
            1 - (self.data['poses']['path'].poses[-1].orientation.x *
                 marker.pose.orientation.x +
                 self.data['poses']['path'].poses[-1].orientation.y *
                 marker.pose.orientation.y +
                 self.data['poses']['path'].poses[-1].orientation.z *
                 marker.pose.orientation.z +
                 self.data['poses']['path'].poses[-1].orientation.w *
                 marker.pose.orientation.w) ** 2
        # Check if current pose is within a tolerance of the last pose in
        # the array
        if dist_xyz <= tol_val and dist_xyzw <= tol_val:
            # Remove the pose from the array
            self.data['poses']['path'].poses = \
                self.data['poses']['path'].poses[:-1]
            Me.info_message("Previous point removed from path.")
            success = True
        else:
            # Tell the user if pose isn't the last pose in the array
            Me.info_message("Move back to the location of the last "
                            "added point in order to remove it.")
        return success

    def get_inspection(self, client, inspection):
        """Call the appropriate action server and get the inspection
        parameters."""
        found = False
        inspection = SurfaceVFGoal(inspection)
        try:
            client.send_goal(inspection)
            client.wait_for_result(Duration.from_sec(5.0))
            result = client.get_result()
            if result.x_size > 0:
                found = True
                self.inspection['pose_list'] = result.pose_array
                self.inspection['x_size'] = result.x_size
                self.inspection['z_size'] = result.z_size
                self.inspection['y_array'] = result.y_array
        except AttributeError:
            found = False

        return found

    def add_pose(self):
        """Convert the frame id of the pose and add it to the array."""
        base_pose = PoseStamped()
        try:
            # Convert pose to base frame
            self.data['poses']['marker'].header.stamp = \
                self.tfl.getLatestCommonTime(self.params['world'],
                                             self.data['poses']
                                             ['marker'].header.frame_id)
            base_pose = self.tfl.transformPose(self.params['world'],
                                               self.data['poses']['marker'])
        except (TfE, LookupException, ConnectivityException):
            Me.error_message("Error transforming pose " +
                             self.data['poses']['marker'].header.frame_id)

        self.data['poses']['path'].header.frame_id = self.params['world']
        self.data['poses']['path'].header.stamp = Time.now()
        self.data['poses']['path'].poses.append(deepcopy(base_pose.pose))

        Me.info_message(self.data['poses']['path'])
        return

    def move_marker(self, key, command):
        """Adjust the location of the feedback marker."""
        if self.check_array_exit(key=key, command=command):
            if key == 'x' and \
                    self.inspection['y_array'][self.data['x']
                                               ['current']] - 1 != \
                    self.data['y']['limit']:
                Me.info_message('VF size change caused by depth change.')
                # Update VF array limits
                self.data['y']['limit'] = int(self.inspection['y_array']
                                              [self.data['x']['current']] - 1)
                # If the number of inspection points across drops
                # below 5 (to 3) move to center, otherwise update
                #  to closest location and reset the limit
                if self.inspection['y_array'][self.data['x']['current']] < 5:
                    self.data['y']['current'] = 1
                else:
                    self.data['y']['current'] = \
                        int(float(self.data['y']['current']) /
                            float(self.data['y']['limit']) *
                            (self.inspection['y_array']
                             [self.data['x']['current']]))

            self.update_current_pose(string='marker')
            self.update_markers()
            self.adjust_camera()
            success = True
        else:
            success = False
        return success

    def check_array_exit(self, key, command):
        """Check to make sure the marker isn't leaving the array."""
        success = False
        if 0 <= self.data[key]['current'] + self.data[key][command] <= \
                self.data[key]['limit']:
            self.data[key]['current'] += self.data[key][command]
            success = True
        elif not 0 <= self.data[key]['current'] + self.data[key][command] <= \
                self.data[key]['limit']:
            Me.error_message("Attempting to leave the inspection "
                             "matrix, move a different direction.")
            success = False
        return success

    def update_current_pose(self, string):
        """Convenience function for reassigning the current pose."""
        self.data['poses'][string].header.frame_id = \
            self.inspection['pose_list'][0].header.frame_id
        self.data['poses'][string].header.stamp = Time.now()

        array_location = (self.data['y']['limit'] + 1) * \
            self.data['z']['current'] + self.data['y']['current']

        self.data['poses'][string].pose = \
            self.inspection['pose_list'][self.data['x']['current']
                                         ].poses[array_location]
        return self.data

    def update_markers(self):
        """Replace VF visualization markers."""
        self.markers.delete_array(marker_id_start=1001,
                                  pose_array=self.inspection['pose_list'
                                                             ][self.data['x']
                                                               ['current']])
        sleep(0.01)
        self.markers. \
            small_sphere_pose_array(marker_id_start=1001,
                                    pose_array=self.inspection['pose_list']
                                    [self.data['x']['current']])
        sleep(0.01)
        self.markers.add_small_sphere_stamped(marker_id=1000,
                                              pose=self.data['poses'][
                                                  'marker'])
        return

    def adjust_camera(self):
        """Adjust the location of the camera to behind the camera link."""
        pose = deepcopy(self.data['poses']['marker'])  # PoseStamped()
        eye_pose = deepcopy(pose)
        eye_pose.pose.position.x += 0.60
        eye_pose.pose.position.z += 0.20
        focus_pose = PoseStamped()
        base_eye_pose = PoseStamped()

        try:
            # Convert pose to base frame
            pose.header.stamp = self.tfl. \
                getLatestCommonTime(self.params['world'], pose.header.frame_id)
            focus_pose = self.tfl.transformPose(self.params['world'], pose)
        except (TfE, LookupException, ConnectivityException):
            Me.error_message("Error transforming pose " + pose.header.frame_id)

        try:
            # Convert pose to base frame
            pose.header.stamp = self.tfl. \
                getLatestCommonTime(self.params['world'],
                                    eye_pose.header.frame_id)
            base_eye_pose = self.tfl.transformPose(self.params['world'],
                                                   eye_pose)
        except (TfE, LookupException, ConnectivityException):
            Me.error_message("Error transforming pose " + pose.header.frame_id)

        cam_place = CameraPlacement()
        cam_place.target_frame = self.params['world']
        cam_place.time_from_start = Duration(1)
        # Position of the camera relative to target_frame
        cam_place.eye.header.frame_id = cam_place.target_frame
        cam_place.eye.point = base_eye_pose.pose.position
        # Target_frame-relative point for the focus
        cam_place.focus.header.frame_id = cam_place.target_frame
        cam_place.focus.point = focus_pose.pose.position
        # Target_frame-relative vector that maps to "up" in the view plane.
        cam_place.up.header.frame_id = cam_place.target_frame
        cam_place.up.vector.x = 0
        cam_place.up.vector.y = 0
        cam_place.up.vector.z = 1
        self.pub.publish(cam_place)
        return


def main():
    """Testing function to pass set commands to teleop."""
    init_node("descartes_teleop_testing")
    # Assign inspection parameters for the class
    params = {'world': "/table_link",
              'group': "left_ur5",
              'clear': True,
              'print_messages': True,
              'debug': False}

    teleop = VFNavigation(params=params)
    teleop.define_inspection(client='polar_camera',
                             inspection_name='ar_2_camera')
    for key in ['y+', 'y-', 'z+', 'z-', 'x+', 'x-', 'x+', 'x+']:
        response = teleop.pose_nav(command=key)
        Me.info_message(response)


if __name__ == "__main__":
    main()
