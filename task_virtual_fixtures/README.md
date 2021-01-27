#DISCLAIMER
Some of the code used for this project is not open source as it was developed by a University of Texas student while interning at LANL.
The code in this repository is open source through LANL and a full open source version is being worked on.
This readme will be updated accordingly when this is complete.

# task_virtual_fixtures
Task Virtual Fixture (TVF) data storage and construction pipeline.

# Table of Contents
1. [Manual Installation](#manual-installation)
2. [Test Your Installation](#test-installation) 
3. [Building a TVF](#building-a-tvf)

***

## About
TO BE ADDED LATER.

Maintainer: [Andrew Sharp](asharp@utexas.edu "He just wants peace and quiet.")

****

## Manual Installation<a name="manual-installation" />

1. **First Steps**<br/> 
    - Ensure you have been added to the UT Nuclear Robotics Group on github and have created a catkin workspace.
      ```
      source /opt/ros/kinetic/setup.bash
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws
      catkin build
      source devel/setup.bash
      ```
    - Within the src folder of your workspace, install the task_virtual_fixtures repo:
      ```
      git clone https://github.com/UTNuclearRobotics/task_virtual_fixtures.git
      ```
    
2. **Download the following ROS repositories**<br/>
   Enter the following lines into a terminal to install ROS repositories
   ```
   sudo apt-get install ros-kinetic-moveit-ros-move-group
   sudo apt-get install ros-kinetic-moveit-ros-planning-interface
   sudo apt-get install ros-kinetic-industrial-msgs
   sudo apt-get install ros-kinetic-industrial-robot-simulator
   sudo apt-get install ros-kinetic-rviz-visual-tools
   ```
3. **Install tvf\_interactive**<br/>
   Enter the following lines into a terminal within your catkin workspace's
   src folder
   ```
   git clone https://github.com/UTNuclearRobotics/tvf_interactive.git
   ```

4. **Install move\_interface**
   ```
   git clone https://github.com/UTNuclearRobotics/move_interface.git
   ```

5. **Install netft\_utils**
   ```
   git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git
   ```

6. **Install the Nuclear Robotics Group Forked Descartes Repository**
   ```
   git clone https://github.com/UTNuclearRobotics/descartes.git
   ```

7. **Build your catkin workspace**<br/>
   Enter the following command within your catkin workspace
   ```
   catkin build
   ```

## Test Your Installation<a name="test-installation">
1. **Run the following launch file**
    ```
    roslaunch tvf_construction tvf_construction.launch
    ```

    If you have issues, make sure your catkin workspace is appropriately
    sourced, and that you have performed step 7 in the manual installation
    section.

2. **Run the following launch file**
    ```
    roslaunch tvf_interactive_demos sia20_simulation_imarkers.launch
    ```

    Allow time for this step to load in rviz.

## Building a TVF<a name="building-a-tvf" />

1.  **Add STL file to repository**<br/>
    Create a folder with the SAME NAME as the .stl file in tvf_data/meshes and copy the .stl into it. Ex. tvf_data/meshes/little_boy/little_boy.stl

2.  **Edit the STL conversion script**<br/>
    In tvf_data/meshes/stl_to_obj_pcd.sh, change the name in the second line to your new folder. Ex. "for file in little_boy/* ; do"

3.  **Run STL conversion script in tvf_data/meshes**
    ```
    ./stl_to_obj_pcd.sh
    ```

4.  **Create new surface file**
     - In tvf_data/surfaces, copy a surface_example.yaml file as surface_yournumberhere.yaml, and open the file.
     - Replace any occurance of "surface_example" with surface_yournumberhere.
     - Change the "folder_path" and "name" to correspond to your new folder.
     - Update TVF generation parameters for your model, VALUES ARE UNITLESS.
     - See TVF_interactive for remaining parameters.

5.  **Edit the TVF construction launch file**<br/>
    Edit tvf_construction/launch/tvf_construction.launch file by replacing in "tvf_id" default="20" with "yournumberhere".

6.  **Run the TVF construction pipeline**
    ```
    roslaunch tvf_construction tvf_construction.launch
    ```
    Caution: this function takes a time, upwards of 30 minutes.  If in the readout, the cloud layer size is over 100,000, increase the intralayer parameter in the surface_yournumberhere.yaml file until the cloud layer size is under 100,000.



## Interactive TVFs in RViz<a name="tvf-interactive" />

7.  In terminal, navigate to the top of the workspace, and run 
    ```
    roslaunch tvf_interactive_demos sia20_simulation_imarkers.launch
    ```
    Caution: While the robot model will load rather quickly, it will take a long time to load the interactive model into RVIZ.  If the file size of the .dot file created in your model folder is over 500MB, the file can take over an hour to load, and the sim may need to be run overnight. 

8.  Move the interactive marker to load the points. 
