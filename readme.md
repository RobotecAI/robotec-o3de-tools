# robotec-o3de-tools

This repository is a collection of useful gems that are usable in O3DE.
Those gems are open source, but refer to the license of every gem.
Note that this is not a "Canonical" part of O3DE - those gems are third-party contributions and are not tested against any particular version of the O3DE engine.

# RobotecRecordingTools
Toolset for joystick-controlled cameras and spline animation tools.

# SplineTools
The tools for expanding the usability of the Spline component in O3DE. 
It allows to:
## Load spline from CSV file

Having a CSV file formatted as :
```csv
x,y,z
0.000000,0.000000,0
0.698132,0.642788,0
1.396263,0.984808,0
2.094395,0.866025,0
2.792527,0.342020,0
3.490659,-0.342020,0
4.188790,-0.866025,0
4.886922,-0.984808,0
5.585054,-0.642788,0
6.283185,-0.000000,0
```

You can modify the Spline component in Editor.
Add SplineToolsEditorComponent next to the [Spline component](https://docs.o3de.org/docs/user-guide/components/reference/shape/spline/), locate your CSV file (it needs to exist as a source asset), and click the Load button.
If you switch `Local Coordinates` to true, the component will interpret coordinates as local to entity origin.
![](doc/SplineToolsEditorComponent.png)


# ROS2ScriptIntegration

Simple, but extremely useful tool that exposes ROS 2 subscription/publication to Script Canvas and LUA.
Refer to [readme](Gems/ROS2ScriptIntegration/readme.md)