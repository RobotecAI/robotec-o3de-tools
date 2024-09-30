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

## Using geo-referenced data

The CSV file can contain the following columns: `lat`, `lon`, `alt` where every row contains the WGS-84 coordinate of the spline's node.
It can be loaded to a georeferenced level as explained in [Georeference section of O3DE documentation]
(https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/georeference/).
It is useful for visualizing paths, roads, and other things at the O3DE level.

# ROS2ScriptIntegration

Simple, but extremely useful tool that exposes ROS 2 subscription/publication to Script Canvas and LUA.
Refer to [readme](Gems/ROS2ScriptIntegration/readme.md)


# RobotecWatchdogTools

Minimal dependency Gem that allows to setup runtime checks and prevents starting the Editor/GameLauncher if the requirements are not met
Refer to [readme](Gems/RobotecWatchdogTools/readme.md)


# LevelModificationTools

TBD

# SensorDebug

A tool that allows to adjust frequency, activate and deactive sensor during game mode.

# Smoothing

Gem contains a smoothing component that will mimic the movement of an attached entity with the tracked entity. It offers multiple smoothing methods. It allows the lock Z axis to point up direction.
Useful for robots' movement smoothing.
![alt text](doc/Smoothing.png) 