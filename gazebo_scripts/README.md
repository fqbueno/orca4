Added by fqbueno and jvperez:

# Control using Gazebo only

In a terminal run:
~~~
cd ~/colcon_ws
source src/orca4/setup.bash
gz sim -v 3 -r sand.world
~~~
Open another terminal to use the control scripts:
~~~
cd ~/colcon_ws
source src/orca4/setup.bash
cd src/orca4
~~~
For example:
~~~
gazebo_scripts/down.sh orca4
gazebo_scripts/stop.sh orca4
gazebo_scripts/up.sh orca4
gazebo_scripts/down.sh orca4
~~~
