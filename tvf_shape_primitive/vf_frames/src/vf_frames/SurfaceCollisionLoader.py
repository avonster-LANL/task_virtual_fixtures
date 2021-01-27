#!/usr/bin/env python
"""Class for adding collision models for detected surfaces based on detected
AR tag number."""

from rospy import init_node, is_shutdown, signal_shutdown, sleep, get_param, \
    Time
from geometry_msgs.msg import Pose
from tf import transformations as tfs, TransformListener, TransformBroadcaster, \
    Exception as TfE
from vf_frames.SurfaceInspectionLoader import SurfaceInspectionLoader
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
__version__ = "2.0.1"
__status__ = "Development"
__description__ = """Class for adding collision models for detected surfaces
                  based on detected AR tag number."""


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
                                         filename=surface['file'],
                                         pose_in=pose, frame=frame_name)

        return


def main():
    """Testing function"""
    name = 'Surface_collision_loader'
    init_node(name)
    Me.info_message('Starting ' + name + ' node.')
    # Detect scene objects with known markers and average poses
    suc = SurfaceCollisionLoader()
    Me.info_message(name + ' node initialized.')
    Me.info_message(name + ' node subscriber initialized.')
    while not is_shutdown():
        frames = suc.surface_frames()
        suc.load_surface_collision()
        # Me.info_message('Surface frames detected: ' + str(frames))
        sleep(0.5)

    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()


if __name__ == "__main__":
    main()
