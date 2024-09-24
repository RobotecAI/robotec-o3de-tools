# robotec-o3de-tools

This repository is a collection of useful gems that are usable in O3DE.
Those gems are open source but refer to the license of every gem.
Note that this is not a "Canonical" part of O3DE - those gems are third-party contributions and are not tested against any particular version of the O3DE engine.

# RobotecRecordingTools
Toolset for joystick-controlled cameras and spline animation tools.

# SplineTools
The tools for expanding the usability of the Spline component in O3DE. 
It allows to:
 - Import spline from CSV file
 - Export spline to a CSV file
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

# RobotecSpectatorCamera

A component that allows to look at an entity from 3rd person perspective and to switch camera mode to the free flying mode (to switch mode press the `C` key). It also allows to enable/disable following the target's rotation and to add a vertical offset to change the `look at` point of the target entity.  
![](doc/RobotecSpectatorCamera.png)


# RobotecWatchdogTools

Minimal dependency Gem that allows the setup runtime checks and prevents starting the Editor/GameLauncher if the requirements are not met
Refer to [readme](Gems/RobotecWatchdogTools/readme.md)

# Disable the main view

Deprecated, please use `-console-mode` switch available in o3de 2409. 
The description of said feature is in the original [PR](https://github.com/o3de/o3de/pull/18093).

# LevelModificationTools

The level modification tool contains a component called PrefabVariantEditorComponent. 
This component allows the change a variant of loaded prefab during game mode.
It exposes PrefabVariantRequestsBus to Script Canvas or LUA.

# SensorDebug

A tool that allows the adjusted frequency, activate and deactivate sensor during the game mode.

# Smoothing

Gem contains a smoothing component that will mimic the movement of an attached entity with the tracked entity. It offers multiple smoothing methods. It allows the lock Z axis to point up direction.
Useful for robots' movement smoothing.  
![alt text](doc/Smoothing.png)

# CsvSpawner

Component that spawns prefabs using coordinates stored in CSV file. It supports XYZ format as well as WGS84 coordinate system.   
![](doc/CsvSpawner.png)

## Load object from CSV file

CSV file format for coordinates in XYZ system:
```csv
x	y	z	name
5.1	37	0	object_name
5.1	44	0	object_name
5.1	51	0	object_name
5.1	58	0	object_name
5.1	65	0	object_name
5.1	72	0	object_name
5.1	79	0	object_name

```

CSV file format for coordinates in WGS84 system:
```csv
alt	lat	lon	name
0	12.5896238486642	30.1930592634115	ball
0	12.5896467730289	30.1931152819367	ball
0	12.5896704915715	30.193170552104	    ball
0	12.5896945425591	30.1932257788658	ball
0	12.5897180240288	30.1932813604207	ball
0	12.5897414039641	30.1933370085065	ball
0	12.5897646831551	30.1933927129084	ball

```

# ExposeConsoleToRos

Simple utility Gem that exposes the O3DE console to ROS 2 topics.
It has two std_msgs/msg/String topics:
```
\o3de_console_out
\o3de_console_in
```
Currently `o3de_console_in` is usable only.
The gem functionality is available only in Profile/Debug.

# Pointcloud

A Gem that introduces point clouds to O3DE.
It offers:
- PointcloudFeatureProcessor with public API
- Pointcloud product asset 
- A public API for configuration (`PointcloudConfigurationBus`)

At this moment it accepts [PLY](https://en.wikipedia.org/wiki/PLY_(file_format)) as a source asset.

![](doc/Pointcloud.png)\
Pointcloud asset was obtained from [potree](https://github.com/potree/potree).

# ROS2PoseControl

A utility gem that introduces a way to control robots in simulation as puppets with Pose messages or TFs.
