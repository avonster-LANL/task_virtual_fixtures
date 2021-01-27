#!/usr/bin/env python
"""This class is the motion command action server which takes command signals
from the client and passes them on to robot interface programs for VF step
teleoperation of the robot."""

from rospy import init_node, get_param, get_name, spin, sleep
from actionlib import SimpleActionServer
from vf_nav.msg import VFNavAction, VFNavFeedback, VFNavResult
from VFNavigation import VFNavigation
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
__status__ = "Production"
__description__ = """This class is the vf nav action server which
                  takes command signals from the client and passes them on to
                  robot interface programs for VF step teleoperation of the
                  robot."""


class VFNavServer(object):
    """Takes in motion commands from the action client and passes them on for
    robot control."""
    def __init__(self, name):
        """Initialize variables for the action server and classes needed to
        pass commands on to the robot."""
        # create messages that are used to publish feedback/result
        self._feedback = VFNavFeedback()
        self._result = VFNavResult()
        self._action_name = name
        self._as = SimpleActionServer(self._action_name,
                                      VFNavAction,
                                      execute_cb=self.execute_cb,
                                      auto_start=False)
        self._as.start()

        params = {'world': get_param('~world_frame'),
                  'package': get_param('~package'),
                  'group': get_param('~group_name'),
                  'print_messages': get_param('~print_messages'),
                  'debug': get_param('~debug'),
                  'clear': get_param('~clear'),
                  'wait': get_param('~wait')}
        self.teleop = VFNavigation(params=params)

        self.get_user_input()
        return

    def get_user_input(self):
        """Have operator choose inspection and action client."""
        continue_vf = True
        while continue_vf:
            task = Me.int_prompt(string="Started VF navigation server. \n"
                                        "Exit = 0 \n"
                                        "savy_4000_camera = 1\n"
                                        "pipe_prop = 2\n"
                                        "wall = 3\n"
                                        "barrel = 4\n"
                                        "pipe_1 = 5\n"
                                        "pipe_2 = 6\n"
                                        "pipe_3 = 7\n"
                                        "pipe_4 = 8\n"
                                        "pipe_5 = 9\n"
                                        "ar_99_camera = 99\n"
                                        "ar_101_camera = 101\n")
            if task == 0:
                Me.info_message("User defined exit.")
            elif task == 1:
                client = 'polar_camera'
                inspection_name = 'ar_1_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 2:
                client = 'polar_camera'
                inspection_name = 'ar_2_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 3:
                client = 'cartesian_camera'
                inspection_name = 'ar_3_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 4:
                client = 'polar_camera'
                inspection_name = 'ar_4_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 5:
                client = 'polar_camera'
                inspection_name = 'ar_5_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 6:
                client = 'polar_camera'
                inspection_name = 'ar_6_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 7:
                client = 'polar_camera'
                inspection_name = 'ar_7_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 8:
                client = 'polar_camera'
                inspection_name = 'ar_8_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 9:
                client = 'polar_camera'
                inspection_name = 'ar_9_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 99:
                client = 'cartesian_camera'
                inspection_name = 'ar_99_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            elif task == 101:
                client = 'CAD_camera'
                inspection_name = 'ar_101_camera'
                self.teleop.define_inspection(client=client,
                                              inspection_name=inspection_name)
            else:
                Me.info_message("Unknown input, shutting down.")
            choice = Me.int_prompt(string="Continue testing?\n"
                                   "Exit = 0 \n"
                                   "Continue = 1\n")
            if choice == 0:
                continue_vf = False
            elif choice == 1:
                continue
            else:
                Me.info_message("Unknown input, shutting down.")
        return

    def execute_cb(self, data):
        """Execute the callback and pass the data command on to
        descartes teleop."""
        # Me.info_message("Executing command")
        self._feedback.feedback = False
        self._as.publish_feedback(self._feedback)
        response = self.teleop.vf_nav(command=data.command)
        if response:
            Me.info_message("Move successful")
        else:
            Me.info_message("Move unsuccessful")
        # Finish callback and wait for next
        self._result.result = True
        # Me.info_message('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        return


def main():
    """Start the action server."""
    # Sleep so RViz has time to load
    sleep(5)
    # TODO: pass keyboard interrupts through and exit properly
    init_node("vf_nav_server")
    VFNavServer(get_name())
    spin()

if __name__ == '__main__':
    main()
