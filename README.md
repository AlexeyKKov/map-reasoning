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
<li>-s key provide an opportunity to use experience</li>
</ol>
<hr>
<b><p><strong>Tested on</strong>:<br>
blocksworld, logistics, spatial tasks</p></b> <br>

<b>Test variants:</b>

benchmarks/mapddl/blocksworld/ 1 -lt classic

benchmarks/mapddl/logistics/ 1 -lt classic

benchmarks/simple/blocks/ 1 -lt classic

benchmarks/spatial/ 1 -g
<br>
<hr>
<p><strong>Requirements</strong>:<br>
python 3.5+</p>

