 <h2>Description</h2> A simple, but complete state machine example. We highly recommend using this example as a starting point for users state machine projects.<br></br>
 
 <a href="https://reelrbtx.github.io/SMACC_Documentation/master/html/namespacesm__three__some.html">Doxygen Namespace & Class Reference</a> 
 
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

And then run the launch file...

```
roslaunch sm_three_some sm_three_some.launch
```

If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
``` 

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.

