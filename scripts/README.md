<h2><a id="Multiagent_planner_based_on_sign_world_model_0"></a>Multiagent planner based on sign world model.</h2>
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

1. Install Theano and Lasagne:

sudo pip3 install --upgrade https://github.com/Theano/Theano/archive/master.zip
sudo pip3 install --upgrade https://github.com/Lasagne/Lasagne/archive/master.zip

2. Install OpenAI gym:

sudo pip3 install gym

3. Install gym_crumb:

https://github.com/ermekaitygulov/gym-crumb.git

4. Install ROS and Gazebo for python3:

sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps
sudo apt-get install ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install python3-yaml
sudo pip3 install rospkg catkin_pkg

5. Start Gazebo

5. Start mover_server, start mapplanner with spatial task