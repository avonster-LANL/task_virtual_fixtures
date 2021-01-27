#!/usr/bin/env python
"""Class for adding frames for detected surfaces based on detected AR tag
number."""

from rospy import init_node, is_shutdown, signal_shutdown, Duration, Time, \
    sleep, get_param
from geometry_msgs.msg import PoseStamped
from tf import transformations as tfs, TransformListener, TransformBroadcaster, \
    Exception as TfE, LookupException, ConnectivityException
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
__version__ = "2.0.1"
__status__ = "Development"
__description__ = """Class for adding frames for detected surfaces based on
                  detected AR tag number."""


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


def main():
    """Testing function. """
    name = 'Surface_frame_loader'
    init_node(name)
    Me.info_message('Starting ' + name + ' node.')
    # Detect tf frames from known markers
    suf = SurfaceFrameLoader()
    Me.info_message(name + ' node initialized.')
    while not is_shutdown():
        suf.get_ar_frames()
        suf.load_surface_frames()
        sleep(0.50)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()


if __name__ == "__main__":
    main()
