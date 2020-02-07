 <img src="https://github.com/reelrbtx/SMACC/blob/master/smacc_sm_reference_library/sm_calendar_week/docs/smacc_state_machine_20200206-003738.dot.svg" width="950" align="center" border="10"/> 
 
 <h2>Description</h2> A simple, but complete state machine example. We highly recommend using this example as a starting point for users state machine projects.<br></br>
 
 <a href="https://github.com/reelrbtx/SMACC/raw/master/smacc_sm_reference_library/sm_calendar_week/docs/smacc_state_machine_20200206-004026.dot.pdf"> Download PDF</a>
 
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
roslaunch sm_calendar_week sm_calendar_week.launch
```

