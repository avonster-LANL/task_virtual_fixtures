#!/usr/bin/env python
"""This class is designed to build VF for known Polar and Cartesian surfaces
marker with AR tags."""

from math import pi
from copy import deepcopy
from numpy import linspace, cos, sin
from rospy import init_node, is_shutdown, signal_shutdown, sleep, get_param, \
    Subscriber, Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import transformations as tfs, TransformListener, TransformBroadcaster
from support_utilities.MessagesExits import MessagesExits as Me
from support_utilities.ManipulateMarkers import ManipulateMarkers
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
__description__ = """This class is designed to build VF for known Polar and
                  Cartesian surfaces marker with AR tags."""


class SurfaceVFBuilder:
    """Class for adding collision models for detected surfaces based on detected
    AR tag number."""
    def __init__(self, params):
        """Read ROS parameters, set up tf classes, initialize dictionary for
        ar_barcode storage, read surface information from launch files, and
        initialize mesh loader class."""
        self.params = params
        self.tbr = TransformBroadcaster()
        self.tfl = TransformListener()
        self.strs = ['files', 'surface_frames', 'surfaces', 'camera', 'rad',
                     'other']
        sil = SurfaceInspectionLoader(params=self.params)
        self.surface_dict = sil.fill_surface_dict(folder='/surfaces/')
        self.inspection_dict = sil.fill_inspection_dict(folder='/inspections/')
        # Initialize marker classes
        self.markers = ManipulateMarkers()
        return

    def surface_frame_callback(self, data):
        """Build a list of the AR surface frames detected."""
        transform_list = data.transforms
        for transform in transform_list:
            frame_id = transform.child_frame_id
            if 'ar_' in frame_id and 'surface' in frame_id and \
                    frame_id not in self.surface_dict[self.strs[1]]:
                self.surface_dict[self.strs[1]].append(frame_id)
            else:
                # Surface frame already in list
                continue
        # sleep(1.0)
        return self.surface_dict

    def build_vfs(self):
        """Build VF if the detected surface frame corresponds to the nodes
        assigned surface and inspection type."""
        if len(self.surface_dict[self.strs[1]]) != 0:
            inspection_matrix = []
            for surface_frame in self.surface_dict[self.strs[1]]:
                barcode_num = surface_frame.split('_')[1]
                inspection_type = \
                    self.inspection_dict[self.params['inspection_type']]
                for inspection in inspection_type.keys():
                    if barcode_num in inspection:
                        surface_id = 'ar_' + barcode_num + '_surface'
                        inspection_id = 'ar_' + barcode_num + '_' + \
                                        self.params['inspection_type']
                        surface = self.surface_dict[self.strs[2]][surface_id]
                        inspection = inspection_type[inspection_id]
                        if surface['type'] == 'Polar' and \
                                self.params['surface_type'] == 'Polar':
                            inspection_matrix = \
                                self.vf_cylindrical(surface_id=surface_id,
                                                    surface=surface,
                                                    inspection=inspection)
                        elif surface['type'] == 'Cartesian' and \
                                self.params['surface_type'] == 'Cartesian':
                            inspection_matrix = \
                                self.vf_cartesian(surface_id=surface_id,
                                                  surface=surface,
                                                  inspection=inspection)
                        else:
                            # surface type isn't a known type or doesn't match
                            # the provided type
                            continue

            if len(inspection_matrix) != 0:
                if self.params['debug']:
                    Me.info_message(self.params['surface_type'] + ' ' +
                                    self.params['inspection_type'] +
                                    ' publishing marker array.')
                num = int(len(inspection_matrix[0])-1/2)
                # Add visual markers for center of array here only for testing
                self.markers.small_sphere_array_stamped(marker_id_start=1001,
                                                        poses=
                                                        inspection_matrix[num])
        sleep(1.0)
        return

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
        # Create pose stamped in surface frame
        pose_target = PoseStamped()
        pose_target.header.frame_id = surface_id
        pose_target.header.stamp = Time.now()

        for r_val in r_values:
            # Calculate the number of theta steps and ensure it is odd
            steps[2] = int(2 * inspection['bound'] * r_val /
                           inspection['delta_arc'])
            if not steps[2] % 2:
                steps[2] += 1
            # Calculate the list of theta values at specified radius
            theta_values = linspace(-inspection['bound'], inspection['bound'],
                                    num=steps[2])
            matrix = [[PoseStamped() for j in range(steps[2])]
                      for k in range(steps[1])]

            # Fill the matrix
            for k in range(steps[1]):
                for j in range(steps[2]):
                    # Fill x, y, z values
                    pose_target.pose.position.x = r_val * cos(theta_values[j])
                    pose_target.pose.position.y = r_val * sin(theta_values[j])
                    pose_target.pose.position.z = z_values[k]
                    # Calculate quaternion values and point toward object
                    quaternion_values = \
                        tfs.quaternion_from_euler(0.0, 0.0,
                                                  theta_values[j] - pi)
                    # Fill quaternion values
                    pose_target.pose.orientation.x = quaternion_values[0]
                    pose_target.pose.orientation.y = quaternion_values[1]
                    pose_target.pose.orientation.z = quaternion_values[2]
                    pose_target.pose.orientation.w = quaternion_values[3]
                    # Put pose in inspection matrix
                    matrix[k][j] = deepcopy(pose_target)
            # Append matrix to list of matrices
            inspection_matrix.append(matrix)
        return inspection_matrix

    @staticmethod
    def vf_cartesian(surface_id, surface, inspection):
        """Calculate the VF coordinates for the cartesian surface. Make the
        number of points odd in order to use the center as the nominal starting
        location."""
        # Calculate the number of normal steps and ensure it is odd
        steps = [0, 0, 0]
        steps[0] = int((inspection['max_offset'] - inspection['min_offset']) /
                       inspection['delta_x'])
        if not steps[0] % 2:
            steps[0] += 1
        x_values = linspace(surface['depth'] + inspection['min_offset'],
                            surface['depth'] + inspection['max_offset'],
                            num=steps[0])
        # Calculate the number of horizontal steps and ensure it is odd
        steps[1] = int(surface['width'] / inspection['delta_y'])
        if not steps[1] % 2:
            steps[1] += 1
        y_values = linspace(surface['width'] / 2, -surface['width'] / 2,
                            num=steps[1])
        # Calculate the number of vertical steps and ensure it is odd
        steps[2] = int(surface['height'] / inspection['delta_z'])
        if not steps[2] % 2:
            steps[2] += 1
        z_values = linspace(-surface['height'] / 2, surface['height'] / 2,
                            num=steps[2])
        # Start inspection matrix list
        inspection_matrix = []
        # Create pose stamped in surface frame
        pose_target = PoseStamped()
        pose_target.header.frame_id = surface_id
        # Calculate quaternion values and point toward object
        quaternion_values = \
            tfs.quaternion_from_euler(0.0, 0.0, - pi)
        # Fill quaternion values
        pose_target.pose.orientation.x = quaternion_values[0]
        pose_target.pose.orientation.y = quaternion_values[1]
        pose_target.pose.orientation.z = quaternion_values[2]
        pose_target.pose.orientation.w = quaternion_values[3]
        # Fill the inspection matrix
        for x_val in x_values:
            pose_target.pose.position.x = x_val
            # Clear sub matrix
            matrix = [[PoseStamped() for k in range(steps[2])]
                      for j in range(steps[1])]
            for j in range(steps[1]):
                pose_target.pose.position.y = y_values[j]
                for k in range(steps[2]):
                    pose_target.header.stamp = Time.now()
                    pose_target.pose.position.z = z_values[k]
                    # Put pose in inspection matrix
                    matrix[j][k] = deepcopy(pose_target)
            # Append matrix to list of matrices
            inspection_matrix.append(matrix)
        return inspection_matrix


if __name__ == "__main__":
    # Name node for specific surface / inspection type
    NAME = 'VF_Polar_camera'
    init_node(NAME, anonymous=True)
    # Read ROS parameters
    PARAMS = {'debug': get_param('~debug'),
              'world': get_param('~world_frame'),
              'package': get_param('~package'),
              'surface_type': get_param('~surface_type'),
              'inspection_type': get_param('~inspection_type')}
    Me.info_message('Starting ' + NAME + ' for ' + PARAMS['surface_type'] +
                    ' and ' + PARAMS['inspection_type'] + ' node.')

    # Detect scene objects with known markers and average poses
    SVF = SurfaceVFBuilder(params=PARAMS)
    Me.info_message(NAME + ' for ' + PARAMS['surface_type'] +
                    ' and ' + PARAMS['inspection_type'] + ' node initialized.')

    Subscriber('tf', TransformStamped, SVF.surface_frame_callback)
    Me.info_message(NAME + ' for ' + PARAMS['surface_type'] +
                    ' and ' + PARAMS['inspection_type'] +
                    ' node subscriber initialized.')

    while not is_shutdown():
        SVF.build_vfs()
        # sleep(2.0)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()
