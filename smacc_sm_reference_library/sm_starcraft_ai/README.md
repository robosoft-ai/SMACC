 <h2>State Machine Diagram</h2>
 <img src="https://github.com/robosoft-ai/SMACC/blob/master/smacc_sm_reference_library/sm_starcraft_ai/docs/smacc_state_machine_20200302-001012.dot.svg" width="950" align="center" border="10"/>

 <h2>Description</h2> Inspired by StarcraftAI, this state machine provides a state machine shell, that can be easily adapted to AI applications. One would simply insert the AI objective function into the state_reactor of the Observe state.<br></br>

 <a href="https://robosoft-ai.github.io/smacc_doxygen/master/html/namespacesm__starcraft__ai.html">Doxygen Namespace & Class Reference</a>

 <p>For more information see...</p>
  <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=10&ved=2ahUKEwiThZKprvvnAhVLvZ4KHbKFDHkQFjAJegQIBxAB&url=https%3A%2F%2Fychai.uk%2Fslides%2F2019-11-12-AlphaStarII.pdf&usg=AOvVaw0jRIVMd3gdb4fM4mmQ4nG1l">AlphaStar: Grandmaster level in StarCraft II Explained</a>

  <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=2ahUKEwiqtLHorvvnAhUNs54KHZyOBtMQFjAAegQIBhAB&url=https%3A%2F%2Fhci.iwr.uni-heidelberg.de%2Fsystem%2Ffiles%2Fprivate%2Fdownloads%2F1448422913%2Freport_johannes_daub.pdf&usg=AOvVaw1ZfV12L1svm6sYG7Y2E9Wj">A new Star is born - Looking into AlphaStar</a>

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
roslaunch sm_starcraft_ai sm_starcraft_ai.launch
```

<h2>Viewer Instructions</h2>
If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
```

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.
