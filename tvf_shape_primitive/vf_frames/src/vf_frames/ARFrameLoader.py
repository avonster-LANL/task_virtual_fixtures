#!/usr/bin/env python
"""Class for adding frames at detected AR tag locations."""

from math import isnan
from numpy import empty, mean, std
from rospy import init_node, is_shutdown, signal_shutdown, Duration, Time, \
    sleep, get_param
from geometry_msgs.msg import PoseStamped
from tf import transformations as tfs, TransformListener, TransformBroadcaster, \
    Exception as TfE, LookupException, ConnectivityException, \
    ExtrapolationException
from vf_frames.SurfaceInspectionLoader import SurfaceInspectionLoader
from support_utilities.MessagesExits import MessagesExits as Me

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
__description__ = """Class for adding frames at detected AR tag locations."""


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
        sil = SurfaceInspectionLoader(params=self.params)
        self.surface_dict = sil.fill_surface_dict(folder='/surfaces/')
        return

    def ar_check_frames(self):
        """Build a list of the surface AR tags detected."""
        for surface in self.surface_dict['surfaces'].keys():
            ar_marker = surface.split('_')[0] + '_marker_' + \
                        surface.split('_')[1]
            ar_frame = surface.split('_')[0] + '_' + surface.split('_')[1]
            xyz_avg, xyzw_avg = \
                self.get_average_pose(num_to_avg=self.num_to_avg,
                                      ar_marker=ar_marker)
            if not isnan(xyz_avg[0]) and sum(xyzw_avg) != 0.0:
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
                if self.params['debug']:
                    Me.info_message('Transform exception')
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
            # Record values to be averaged
            x_positions[i] = pose.pose.position.x
            y_positions[i] = pose.pose.position.y
            z_positions[i] = pose.pose.position.z
            # switch to Euler angles
            euler_angles = \
                tfs.euler_from_quaternion(
                    [pose.pose.orientation.x,
                     pose.pose.orientation.y,
                     pose.pose.orientation.z,
                     pose.pose.orientation.w])

            roll_orientations[i] = euler_angles[0]
            pitch_orientations[i] = euler_angles[1]
            yaw_orientations[i] = euler_angles[2]
            # Sleep to ensure averaging of different transforms
            sleep(0.02)
        # Average the collected data
        xyz_avg = (self.cut_outliers(x_positions),
                   self.cut_outliers(y_positions),
                   self.cut_outliers(z_positions))
        quaternion_avg = \
            tfs.quaternion_from_euler(self.cut_outliers(roll_orientations),
                                      self.cut_outliers(pitch_orientations),
                                      self.cut_outliers(yaw_orientations))
        xyzw_avg = (quaternion_avg[0], quaternion_avg[1],
                    quaternion_avg[2], quaternion_avg[3])
        return xyz_avg, xyzw_avg

    @staticmethod
    def cut_outliers(data):
        """Remove the outliers from the orientation data. """
        stds = 1
        data_mean = mean(data)
        data_std = std(data)
        if data_std <= 1e-5:
            # No deviation, simulated data
            filtered = data
        else:
            # Deviation, cut out the outliers
            filtered = [val for val in data
                        if (data_mean - stds * data_std <
                            val < data_mean + stds * data_std)]
        return mean(filtered)

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

if __name__ == "__main__":
    NAME = 'AR_frame_loader'
    init_node(NAME)
    Me.info_message('Starting ' + NAME + ' node.')
    # Detect scene objects with known markers and average poses
    ARF = ARFrameLoader(num_to_avg=10)
    Me.info_message(NAME + ' node initialized.')
    while not is_shutdown():
        ARF.ar_check_frames()
        ARF.load_ar_frames()
        sleep(0.25)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()
