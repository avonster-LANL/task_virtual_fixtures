#!/usr/bin/env python
"""This class is designed to build VF for known Polar and Cartesian surfaces
marked with AR tags and provide the VF to other programs through an action
server."""

from os import listdir
from math import pi
from copy import deepcopy
from numpy import linspace, cos, sin
from rospkg import RosPack
from rospy import init_node, get_name, is_shutdown, signal_shutdown, \
    get_param, Time, sleep
from rosbag import Bag
from actionlib import SimpleActionServer
from geometry_msgs.msg import Pose, PoseArray
from tf import transformations as tfs, TransformListener, Exception as TfE
from vf_construction.msg import SurfaceVFAction, SurfaceVFFeedback, \
    SurfaceVFResult
from support_utilities.MessagesExits import MessagesExits as Me
from vf_frames.SurfaceInspectionLoader import SurfaceInspectionLoader

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
__description__ = """This class is designed to build VFs for known Polar and
                  Cartesian surfaces marked with AR tags and provide the VF
                  to other programs through an action server."""


class SurfaceVFServer:
    """Class for adding collision models for detected surfaces based on detected
    AR tag number."""
    def __init__(self, name, params):
        """Start action server, get ROS params, and load possible surfaces and
        inspections."""
        self._as = SimpleActionServer(name, SurfaceVFAction,
                                      execute_cb=self.action_sever_cb,
                                      auto_start=False)
        self._as.start()

        self.params = params
        self.tfl = TransformListener()
        sil = SurfaceInspectionLoader(params=self.params)
        self.surface_dict = sil.fill_surface_dict(folder='/surfaces/')
        self.inspection_dict = sil.fill_inspection_dict(folder='/inspections/')
        self.strs = ['files', 'surface_frames', 'surfaces', 'camera', 'rad',
                     'other']
        return

    def action_sever_cb(self, data):
        """This is the callback for the actions server which receives a string
        for the ar_surface frame."""
        Me.info_message(get_name() + " received request for " + data.ar_tag)
        _feedback = SurfaceVFFeedback()
        _result = SurfaceVFResult()
        _feedback.running = True
        # Send feedback
        self._as.publish_feedback(_feedback)
        if len(self.inspection_dict[self.params['inspection_type']]
               [data.ar_tag]['inspection_matrix']) != 0:
            # Set results
            _result.pose_array = \
                self.inspection_dict[self.params['inspection_type']
                                     ][data.ar_tag]['inspection_matrix']
            _result.y_array = \
                self.inspection_dict[self.params['inspection_type']
                                     ][data.ar_tag]['matrix_size'][2:]
            _result.x_size = \
                self.inspection_dict[self.params['inspection_type']
                                     ][data.ar_tag]['matrix_size'][0]
            _result.z_size = \
                self.inspection_dict[self.params['inspection_type']
                                     ][data.ar_tag]['matrix_size'][1]
        else:
            # Set results
            _result.pose_array = []
            _result.y_array = []
            _result.x_size = 0
            _result.z_size = 0
        # Send results
        self._as.set_succeeded(_result)
        return

    def surface_frames(self):
        """Build a list of the AR surface frames detected."""
        for i in range(0, 244, 1):
            frame = '/ar_' + str(i) + '_surface'
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

    def build_vfs(self):
        """Build VF if the detected surface frame corresponds to the nodes
        assigned surface and inspection type."""
        if len(self.surface_dict[self.strs[1]]) != 0:
            for surface_frame in self.surface_dict[self.strs[1]]:
                barcode_num = surface_frame.split('_')[1]
                inspection_type = \
                    self.inspection_dict[self.params['inspection_type']]
                for inspection in inspection_type.keys():
                    self.build_inspection_dict(barcode_num=barcode_num,
                                               inspection=inspection,
                                               inspection_type=inspection_type)
        # else:
        #     Me.error_message("Known surface dictionary is empty.")
        return self.inspection_dict

    def build_inspection_dict(self, barcode_num, inspection, inspection_type):
        """Build the inspection dictionary with known environmental barcodes."""
        if barcode_num in inspection:
            surface_id = 'ar_' + barcode_num + '_surface'
            inspection_id = 'ar_' + barcode_num + '_' + \
                            self.params['inspection_type']
            surface = self.surface_dict[self.strs[2]][surface_id]
            inspection = inspection_type[inspection_id]
            if surface['type'] == 'Polar' and \
                    self.params['surface_type'] == 'Polar':
                inspection_matrix, matrix_size = \
                    self.vf_cylindrical(surface_id=surface_id, surface=surface,
                                        inspection=inspection)
                # For each inspection add inspection matrix to a dictionary
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['inspection_matrix'] = \
                    deepcopy(inspection_matrix)
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['matrix_size'] = \
                    deepcopy(matrix_size)
                success = True
            elif surface['type'] == 'Cartesian' and \
                    self.params['surface_type'] == 'Cartesian':
                inspection_matrix, matrix_size = \
                    self.vf_cartesian(surface_id=surface_id,
                                      surface=surface,
                                      inspection=inspection)
                # For each inspection add inspection matrix to a dictionary
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['inspection_matrix'] = \
                    deepcopy(inspection_matrix)
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['matrix_size'] = \
                    deepcopy(matrix_size)
                success = True
            elif surface['type'] == 'CAD' and \
                    self.params['surface_type'] == 'CAD':
                inspection_matrix, matrix_size = \
                    self.vf_cad(surface_id=surface_id,
                                surface=surface,
                                inspection=inspection)
                # For each inspection add inspection matrix to a dictionary
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['inspection_matrix'] = \
                    deepcopy(inspection_matrix)
                self.inspection_dict[self.params['inspection_type']
                                     ][inspection_id]['matrix_size'] = \
                    deepcopy(matrix_size)
                success = True
            else:
                success = False
                # Me.error_message("Surface type isn't a known type or doesn't "
                #                  "match the provided type.")
        else:
            success = False
        return success

    @staticmethod
    def vf_cylindrical(surface_id, surface, inspection):
        """Calculate the VF coordinates for the polar surface. Make the number
        of points odd in order to use the center as the nominal starting
        location."""
        # Calculate the number of radial steps and ensure it is odd
        steps = [0, 0, 0]
        steps[0] = int((inspection['max_offset'] - inspection['min_offset']) /
                       inspection['delta_offset'])
        if not steps[0] % 2:
            steps[0] += 1
        r_values = linspace(surface['radius'] + inspection['min_offset'],
                            surface['radius'] + inspection['max_offset'],
                            num=steps[0])
        # Calculate the number of vertical steps and ensure it is odd
        steps[1] = int(surface['height'] / inspection['delta_z'])
        if not steps[1] % 2:
            steps[1] += 1
        z_values = linspace(-surface['height'] / 2, surface['height'] / 2,
                            num=steps[1])

        # Start inspection matrix list
        inspection_matrix = []
        matrix_size = [steps[0], steps[1]]
        for r_val in r_values:
            # Calculate the number of theta steps and ensure it is odd
            steps[2] = int(2 * inspection['bound'] * r_val /
                           inspection['delta_arc'])
            if not steps[2] % 2:
                steps[2] += 1
            # Calculate the list of theta values at specified radius
            theta_values = linspace(-inspection['bound'], inspection['bound'],
                                    num=steps[2])
            pose_array = PoseArray()
            pose_array.header.frame_id = surface_id
            pose_array.header.stamp = Time.now()
            # Fill the matrix
            for k in range(steps[1]):
                for j in range(steps[2]):
                    # Create pose stamped in surface frame
                    current_pose = Pose()
                    # Fill x, y, z values
                    current_pose.position.x = r_val * cos(theta_values[j])
                    current_pose.position.y = r_val * sin(theta_values[j])
                    current_pose.position.z = z_values[k]
                    # Calculate quaternion values and point toward object
                    quaternion_values = \
                        tfs.quaternion_from_euler(0.0, 0.0,
                                                  theta_values[j] - pi)
                    # Fill quaternion values
                    current_pose.orientation.x = quaternion_values[0]
                    current_pose.orientation.y = quaternion_values[1]
                    current_pose.orientation.z = quaternion_values[2]
                    current_pose.orientation.w = quaternion_values[3]
                    pose_array.poses.append(deepcopy(current_pose))
            # Append matrix to list of matrices
            inspection_matrix.append(deepcopy(pose_array))
            matrix_size.append(steps[2])
        return inspection_matrix, matrix_size

    @staticmethod
    def vf_cartesian(surface_id, surface, inspection):
        """Calculate the VF coordinates for the cartesian surface. Make the
        number of points odd in order to use the center as the nominal starting
        location."""
        # Calculate the number of normal steps
        steps = [0, 0, 0]
        steps[0] = int((inspection['max_offset'] - inspection['min_offset']) /
                       inspection['delta_x'])
        x_values = linspace(inspection['min_offset'], inspection['max_offset'],
                            num=steps[0])
        # Calculate the number of vertical steps
        steps[1] = int(surface['height'] / inspection['delta_z'])
        z_values = linspace(-surface['height'] / 2, surface['height'] / 2,
                            num=steps[1])
        # Calculate the number of horizontal steps
        steps[2] = int(surface['width'] / inspection['delta_y'])
        y_values = linspace(-surface['width'] / 2, surface['width'] / 2,
                            num=steps[2])
        # Start inspection matrix list
        inspection_matrix = []
        matrix_size = [steps[0], steps[1], steps[2]]
        for x_val in x_values:
            # Create pose array and fill header and time
            pose_array = PoseArray()
            pose_array.header.frame_id = surface_id
            pose_array.header.stamp = Time.now()
            for z_val in z_values:
                for y_val in y_values:
                    # Create pose
                    current_pose = Pose()
                    # Fill the inspection matrix
                    current_pose.position.x = x_val
                    current_pose.position.y = y_val
                    current_pose.position.z = z_val
                    # Calculate quaternion values and point toward object
                    quaternion_values = \
                        tfs.quaternion_from_euler(0.0, 0.0, -pi)
                    # Fill quaternion values
                    current_pose.orientation.x = quaternion_values[0]
                    current_pose.orientation.y = quaternion_values[1]
                    current_pose.orientation.z = quaternion_values[2]
                    current_pose.orientation.w = quaternion_values[3]
                    # Put pose in inspection matrix
                    pose_array.poses.append(deepcopy(current_pose))
            # Append pose array to list of pose arrays
            inspection_matrix.append(deepcopy(pose_array))
            matrix_size.append(steps[2])
        return inspection_matrix, matrix_size

    def vf_cad(self, surface_id, surface, inspection):
        """Load the VF coordinates for CAD surfaces."""
        ros_pack = RosPack()
        folder_path = ros_pack.get_path(self.params['package']) + \
            "/meshes/" + surface['file'].split("/")[1] + "/"
        # Start inspection matrix list
        inspection_matrix = []
        matrix_size = [1]
        for filename in listdir(folder_path):
            if '.bag' in filename:
                # Me.info_message(filename)
                bag = Bag(folder_path + filename)
                for topic, msg, time in bag.read_messages(topics=['PoseArray']):
                    continue
                    # print msg
                msg.header.frame_id = surface_id
                msg.header.stamp = Time.now()
                # Append pose array to list of pose arrays
                inspection_matrix.append(deepcopy(msg))
                matrix_size.append(len(msg.poses))
                bag.close()
            else:
                continue
        return inspection_matrix, matrix_size


def main():
    """Testing function."""
    # Name node for specific surface / inspection type
    name = 'VF_Polar_camera_server'
    init_node(name)
    # Read ROS parameters
    params = {'debug': get_param('~debug'),
              'world': get_param('~world_frame'),
              'package': get_param('~package'),
              'surface_type': get_param('~surface_type'),
              'inspection_type': get_param('~inspection_type')}
    Me.info_message('Starting ' + name + ' for ' + params['surface_type'] +
                    ' and ' + params['inspection_type'] + ' node.')

    # Detect scene objects with known markers and average poses
    svf = SurfaceVFServer(name=get_name(), params=params)
    Me.info_message(name + ' for ' + params['surface_type'] +
                    ' and ' + params['inspection_type'] + ' node initialized.')

    while not is_shutdown():
        frames = svf.surface_frames()
        svf.build_vfs()
        # Me.info_message('Surface frames detected: ' + str(frames))
        sleep(0.5)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()
    return


if __name__ == "__main__":
    main()
