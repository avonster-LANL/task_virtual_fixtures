#!/usr/bin/env python
"""This class takes in control inputs and sends them to vf nav action
server to be used for sending commands to the robot."""

from sys import stdin
from time import time
from select import select
from termios import tcgetattr, tcsetattr, TCSADRAIN
from tty import setraw
from rospy import init_node, get_param, Subscriber, sleep, Duration, \
    signal_shutdown, is_shutdown
from actionlib import SimpleActionClient
from sensor_msgs.msg import Joy
from vf_nav.msg import VFNavAction, VFNavGoal
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
__description__ = """This class takes in control inputs and sends them to motion
                  command action server to be used for sending commands to the
                  robot."""


class JoyStick:
    """Class for publishing vf navs with the use of a joystick."""
    # TODO: no joystick at LANL, not tested
    def __init__(self):
        """Initialize class variables for joystick dead man vf navs."""
        Me.info_message("Starting a keyboard vf nav client.")
        Subscriber("joy", Joy, self.joy_callback)
        # Deadman variables
        self.joy = Joy()
        self.deadman_pressed = False
        self.release_deadman = False
        self.motion = False
        # Key bindings for control
        self.x_bindings = {'0': "x-", '2': "x+"}
        self.y_bindings = {'1': "y-", '3': "y+"}
        self.z_bindings = {'5': "z-", '4': "z+"}
        return

    def run(self):
        """Send vf navs with a dead man button."""
        msg = """Control_interface!
                ---------------------------
                '0': "x-", '2': "x+"
                '1': "y-", '3': "y+"
                '5': "z-", '4': "z+"
                Press ` to quit
                ---------------------------\n"""
        Me.info_message(msg)

        while not is_shutdown():
            # Wait for the joystick data to be published if necessary
            try:
                buttons = self.joy.buttons

                if buttons[8] == 1:
                    # Stop signal control values
                    self.deadman_pressed = True
                    self.release_deadman = True

                    if 1 in buttons[0:6]:
                        move = self.movement(msg=msg, buttons=buttons)
                else:
                    self.deadman_pressed = False

                # If deadman released send stop command
                if self.release_deadman and not self.deadman_pressed:
                    # Fill and send goal then wait for response
                    goal = VFNavGoal('stop')
                    CLIENT.send_goal(goal)
                    CLIENT.wait_for_result(Duration.from_sec(5.0))

                # If motion and move isn't bank, publish command
                if self.motion and move != '':
                    # Fill and send goal then wait for response
                    goal = VFNavGoal(move)
                    CLIENT.send_goal(goal)
                    CLIENT.wait_for_result(Duration.from_sec(5.0))
                    self.motion = False

                if PRINT_MESSAGES:
                    Me.info_message(move)
                sleep(SLEEP_TIME)

            except IndexError:
                # Inform user and sleep to wait for joystick data
                Me.info_message("Waiting for joystick data.\n")
                sleep(2)
            except KeyboardInterrupt:
                Me.info_message("Shutting down keyboard command interface.")
                signal_shutdown("KeyboardInterrupt")
                Me.shutdown_node()
        Me.info_message("Shutting down keyboard command interface.")
        signal_shutdown("KeyboardInterrupt")
        Me.shutdown_node()

    def movement(self, msg, buttons):
        """Check movement keys and pass the move back."""
        self.motion = True
        # Check movement keys
        move = ''
        for i in range(len(buttons[0:6])):
            if str(buttons[i]) in self.x_bindings.keys():
                move = self.x_bindings[str(i)]
            elif str(buttons[i]) in self.x_bindings.keys():
                move = self.x_bindings[str(i)]
            elif str(buttons[i]) in self.x_bindings.keys():
                move = self.x_bindings[str(i)]
            else:
                Me.info_message("Improper input")
                Me.info_message(msg)
        return move

    def joy_callback(self, data):
        """Callback writes new data and returns self.joy when it is
        published."""
        self.joy = data
        return self.joy


class KeyBoard:
    """Class for publishing vf nav with the use of a keyboard."""
    def __init__(self):
        """Initialize class variables for keyboard vf nav."""
        Me.info_message("Starting a keyboard vf nav client.")
        # Key bindings for control
        self.x_bindings = {'1': 'x-', '2': 'x+'}
        self.y_bindings = {'4': 'y-', '5': 'y+'}
        self.z_bindings = {'7': 'z-', '8': 'z+'}
        return

    def run(self):
        """Send vf nav with a based on keyboard input."""
        msg = """Control_interface!
                ---------------------------
                '1': "x-", '2': "x+"
                '4': "y-", '5': "y+"
                '7': "z-", '8': "z+"
                Press ` to quit
                ---------------------------\n"""
        Me.info_message(msg)

        try:
            while not is_shutdown():
                key = self.get_key()
                # Check exit key
                if key == '`':
                    break
                # Check movement keys
                move = ''
                if key in self.x_bindings.keys():
                    move = self.x_bindings[key]
                elif key in self.y_bindings.keys():
                    move = self.y_bindings[key]
                elif key in self.z_bindings.keys():
                    move = self.z_bindings[key]
                elif key != '':
                    Me.info_message("Improper input")
                    Me.info_message(msg)
                else:
                    continue

                # If move isn't bank, publish
                if move != '':
                    # Fill and send goal then wait for response
                    goal = VFNavGoal(command=move)
                    if PRINT_MESSAGES:
                        Me.info_message(str(goal))
                    CLIENT.send_goal(goal)
                    now = time()
                    if PRINT_MESSAGES:
                        Me.info_message("Waiting 5 seconds for result.\n")
                    CLIENT.wait_for_result(Duration.from_sec(10.0))
                    if time() > now + 5:
                        Me.error_message("Waiting timed out. Exiting.")
                        break
                    if PRINT_MESSAGES:
                        Me.info_message("Process complete.")
                    if PRINT_MESSAGES:
                        Me.info_message(move)

                sleep(SLEEP_TIME)

            Me.info_message("Shutting down keyboard command interface.")
            signal_shutdown("KeyboardInterrupt")
            Me.shutdown_node()

        except KeyboardInterrupt:
            Me.info_message("Shutting down keyboard command interface.")
            signal_shutdown("KeyboardInterrupt")
            Me.shutdown_node()

    @staticmethod
    def get_key():
        """Get keyboard input."""
        setraw(stdin.fileno())
        rlist, _, _ = select([stdin], [], [], 0.1)
        if rlist:
            key = stdin.read(1)
        else:
            key = ''

        tcsetattr(stdin, TCSADRAIN, tcgetattr(stdin))
        return key


if __name__ == '__main__':
    init_node("vf_nav_client", anonymous=True)
    CLIENT = SimpleActionClient("vf_nav_server",
                                VFNavAction)
    Me.info_message("Waiting for server.")
    CLIENT.wait_for_server()
    # Operating parameters
    SLEEP_TIME = 0.001
    PRINT_MESSAGES = False
    INTERFACE_TYPES = {'keyboard': "key",
                       'joystick': "joy"}

    INTERFACE_TYPE = get_param('~interface_type')
    if INTERFACE_TYPE == INTERFACE_TYPES['keyboard']:
        TASK = KeyBoard()
    elif INTERFACE_TYPE == INTERFACE_TYPES['joystick']:
        TASK = JoyStick()
    else:
        Me.info_message("Selected command interface unavailable")
        TASK = ''
    TASK.run()
