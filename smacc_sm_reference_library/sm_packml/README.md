 <h2>State Machine Diagram</h2>
<img src="https://github.com/robosoft-ai/SMACC/blob/master/smacc_sm_reference_library/sm_packml/docs/smacc_state_machine_20200205-104849.dot.svg" width="950" align="center" border="10"/>

<h2>Description</h2> This example implements the 17 state, PackML Interface State Model, shown on page 17, Fig.9 of the <a href="http://omac.org/wp-content/uploads/2016/11/PackML_Unit_Machine_Implementation_Guide-V1-00.pdf">PackML Unit Machine Implementation Guide</a>.<br></br>

 <a href="https://robosoft-ai.github.io/smacc_doxygen/master/html/namespacesm__packml.html">Doxygen Namespace & Class Reference</a>

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
roslaunch sm_packml sm_packml.launch
```

<h2>Viewer Instructions</h2>
If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
```

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.
