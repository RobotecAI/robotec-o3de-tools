## Viewport Streamer Gem

This Gem provides sensor, which streams application's current viewport on ROS2 topic.

## Features

This Gem enables sensor which publishes data on `sensor_msgs::msg::Image` ROS2 topic. Such data contain entire frame displayed in application's current viewport. 

## Prerequisites

- `o3de` project
- [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2)

## Integration

In Editor (builded with `ViewportStreamer` Gem), add `ViewportStreamer Component` in level entity of particular level. If needed, change default sensor configuration. By default, sensor's topic name is `/viewport`.

## Interaction

Frames streamed on `/viewport` might be displayed e.g. using `rviz2`:

![Viewport stream displayed in rviz2](./Docs/images/display_rviz2.png)
