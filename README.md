<h2>Multiagent planner based on sign world model.</h2>
<hr>
<p><strong>To start planner</strong>:<br>
Start <code>mapplanner.py</code> with arguments</p>
<ol>
<li>Path to benchmark folder (i.e. benchmarks/mapddl/logistics00/city_trucks_constr/<br>
<strong>OR</strong> benchmarks/spatial/). Folder should contain task+number file and domain file.</li>
<li>Number of a task in integer format (i.e. 1,2,3…).</li>
<li>Logic type with key <strong><em>-lt</em></strong>. The current implementation of the planner supports only classical and spatial logic (i.e. -lt spatial, -lt classic)</li>
<li>Gazebo implementation with key <strong><em>-g</em></strong>.</li>
<li>-s / -w keys provide an opportunity to use experience (i.e. -s key leads to saving plan like an experience action and -w leads to load previous plans)</li>
</ol>
<hr>
<b><p><strong>Tested on</strong>:<br>
blocksworld, logistics00, spatial tasks</p></b> <br>

<b>Test variants:</b>

benchmarks/mapddl/blocksworld/signdomtask/task3constr/ 33 -lt classic

benchmarks/mapddl/logistics00/city_trucks_constr/ 1 -lt classic

benchmarks/simple/blocks/ 01 -lt classic

benchmarks/spatial/ 1
<br>
<hr>
<p><strong>Requirements</strong>:<br>
python 3.5+</p>


If Gazebo visualization for spatial tasks is needed:

1. Install ROS and Gazebo for python3:
<i>
sudo apt-get install ros-kinetic-desktop-full <br>
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps
sudo apt-get install ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator <br>
sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs <br>
sudo apt-get install ros-kinetic-position-controllers <br>
sudo apt-get install ros-kinetic-controller-manager <br>
sudo apt-get install ros-kinetic-position-controllers <br>
sudo apt-get install ros-kinetic-effort-controllers <br>
sudo apt-get install ros-kinetic-joint-state-controller <br>
sudo apt-get install ros-kinetic-gazebo-ros-control <br>
sudo apt-get install python3-yaml <br>
sudo pip3 install rospkg catkin_pkg <br>
</i>
2. Install Crumb project:

<i>https://github.com/CRUMBproject</i>

3. Move the current project to:

<i>crumb/src</i>

4. Build Crumb:

in /crumb directory: <i>catkin_make</i>

5. Install Theano and Lasagne:

<i>sudo pip3 install --upgrade https://github.com/Theano/Theano/archive/master.zip</i> <br>
<i>sudo pip3 install --upgrade https://github.com/Lasagne/Lasagne/archive/master.zip</i> <br>

6. Install OpenAI gym:

<i>sudo pip3 install gym</i>

7. Install gym_crumb:

<i>https://github.com/ermekaitygulov/gym-crumb.git</i>

8. Change pick_place.world file
in <br>
<i>crumb/src/ROS/crumb/crumb_gazebo/worlds/ <br><i>
to <br>
<i>pick_place.world</i> in benchmarks/spatial/

9. Start Gazebo with crumb_gazebo module:

In terminal:

cd ~/crumb </br>
source devel/setup.bash </br>
<i>roslaunch crumb_gazebo crumb_pick_place.launch</i>

10. Start mapplanner with spatial task and key -g

<i>benchmarks/spatial/ 1 -g</i>
