#!/usr/bin/env python
"""Class for reading in information from surface and inspection launch files."""

from os import listdir
from rospkg import RosPack
from rospy import get_param

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
__description__ = """Class for filling surface and inspection information from
                  launch files in specified package directories."""


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
                self.surface_dict[self.strs[2]][surface]['file'] \
                    = '/' + self.surface_dict[self.strs[2]][surface]['name'] +\
                      '/' + self.surface_dict[self.strs[2]][surface]['name'] +\
                      '.stl'
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
