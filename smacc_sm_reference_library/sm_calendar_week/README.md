  <h2>State Machine Diagram</h2>
 <img src="https://github.com/robosoft-ai/SMACC/blob/master/smacc_sm_reference_library/sm_calendar_week/docs/smacc_state_machine_20200206-003738.dot.svg" width="950" align="center" border="10"/>

 <h2>Description</h2> Another simple, but complete state machine example that uses the ros_timer_client & keyboard client to iterate through the states, which correspond to the days of the week. This example is another excellent starting point for new users state machine projects.<br></br>

 <a href="https://robosoft-ai.github.io/smacc_doxygen/master/html/namespacesm__calendar__week.html">Doxygen Namespace & Class Reference</a>

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
roslaunch sm_calendar_week sm_calendar_week.launch
```

 <h2>Viewer Instructions</h2>
If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
```

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.
