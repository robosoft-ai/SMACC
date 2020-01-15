 <h2>Description</h2> A completely minimal state machine example.<br></br>
<a href="https://reelrbtx.github.io/SMACC_Documentation/master/html/namespacesm__atomic.html">Doxygen Namespace & Class Reference</a>
 
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
roslaunch sm_atomic sm_atomic.launch
```

Or, you can run the lightweight demo...

```
roslaunch sm_atomic sm_atomic.launch sm_xterm:=nice server_nodes_xterms:=nice show_gz_client:=false show_smacc_viewer:=false
```

This launch command starts the process by hiding the xterminals for all the servers, the state machine, the smacc_viewer and the gazebo client.
 
