# Pose control

You can use the `ROS2PoseControl` component to control the tractor or any arbitrary entity via ROS 2 position or tf messages.

## Adding component
Attach it to the the entity you wish to control, at the to level or on the same as rigid body if one is present, by clicking 'add component' and selecting it from the list.

## Configuration

After adding the component, you should see a window like this:

![Pose control UI](../Images/pose_control_ui.png)

You can use this component in following modes:

1. Using `PoseStamped` messages - component listens to the selected topic of type [`geometry_msgs::PoseStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html) and changes transform of the entity based on Pose recived. The timestamp of the `PoseStamped` message is ignored and position is applied based on message arival order.

2. Using the [tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) messages - component will look for transform frame specified by you and use it to transform the entity.

Option | Description
--------| -----
Is Tracking | Turns this component on/off 
Tracking Mode | Selects the tracking mode
Topic for goal message | Standard topic configuration (only used in mode PoseMessages)
Target Frame | Frame of the tractor (only used in mode TF2)
Reference Frame | Frame of the origin (only used in mode TF2)

### Disabling physics based movement

This component it designed to take full controll over movment of the entity. You might need to set [Rigid Body](https://docs.o3de.org/docs/user-guide/components/reference/physx/rigid-body/) type to kinematic. 

## ImGui live options

Using the ~ key, you can pull up ImGui options. There, you will see two toggles using which you can modify the `Is Tracking` and `Tracking mode` settings live.