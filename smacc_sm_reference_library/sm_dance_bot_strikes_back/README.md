 <h2>State Machine Diagram</h2>
<img  src="https://github.com/reelrbtx/SMACC/blob/master/smacc_sm_reference_library/sm_dance_bot_strikes_back/docs/smacc_state_machine_20200222-122229.dot.svg" width="950" align="center" border="10"/>

<h2>Description</h2> A full-featured state machine example, that highlights the capabilities of SMACC & the ROS Navigation Stack via the MoveBaseZ Client.<br></br>

<a href="https://reelrbtx.github.io/SMACC_Documentation/master/html/namespacesm__dance__bot__strikes__back.html">Doxygen Namespace & Class Reference</a>

<h2>Build Instructions</h2>
Before you build, make sure you've installed all the dependencies...

```
rosdep install --from-paths src --ignore-src -r -y 
```

Then you build with either catkin build or catkin make...

```
catkin build
```
<h2>Operating Instructions</h2>
After you build, remember to source the proper devel folder...

```
source ~/catkin_ws/devel/setup.bash
```

And then run the launch file for the full gazebo demo...

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch
```

Or, you can run the lightweight demo...

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch sm_xterm:=nice server_nodes_xterms:=nice show_gz_client:=false show_smacc_viewer:=false
```

This launch command starts the process by hiding the xterminals for all the servers, the state machine, the smacc_viewer and the gazebo client.

<h2>Viewer Instructions</h2>
If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
``` 

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.


