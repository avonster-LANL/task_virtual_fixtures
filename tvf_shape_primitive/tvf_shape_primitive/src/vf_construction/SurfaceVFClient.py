#!/usr/bin/env python
"""Testing class for SurfaceVFServer."""

from rospy import init_node, Duration, signal_shutdown
from actionlib import SimpleActionClient
from non_contact_vf.msg import SurfaceVFAction, SurfaceVFGoal
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
__description__ = """Testing class for SurfaceVFServer."""


class SurfaceVFClient:
    """Testing class for SurfaceVFServer."""
    def __init__(self):
        """Testing class for SurfaceVFServer."""
        client = SimpleActionClient("VF_Polar_camera_server", SurfaceVFAction)
        Me.info_message("Waiting for server.")
        client.wait_for_server()
        goal = SurfaceVFGoal('ar_16_camera')
        client.send_goal(goal)
        client.wait_for_result(Duration.from_sec(5.0))
        result = client.get_result()
        Me.info_message('Number of pose arrays: ' + str(len(result.pose_array)))

        # Add visual markers for center of array here only for testing
        # markers.small_sphere_pose_array(marker_id_start=1001,
        #                                 pose_array=
        #                                 self.inspection_dict[self.params
        # ['inspection_type']]['inspection_matrix'][0])
        return


if __name__ == "__main__":
    # Name node for specific surface / inspection type
    NAME = 'VF_Polar_camera_client'
    init_node(NAME, anonymous=True)
    SVF = SurfaceVFClient()
    # Exit the program to not cause an error.
    signal_shutdown("KeyboardInterrupt")
    Me.shutdown_node()
