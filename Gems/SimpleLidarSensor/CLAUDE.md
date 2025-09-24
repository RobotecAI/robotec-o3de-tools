# SimpleLidarSensor Gem - Development Guide

## Overview
SimpleLidarSensor is an O3DE gem that provides LIDAR sensor simulation using multiple camera views to generate point cloud data published via ROS2.

## Architecture
- **Multi-camera setup**: Uses 3 cameras (ViewCount = 3) arranged to provide 360° coverage
- **Horizontal FOV**: 120° per camera (360°/3)
- **Output**: Publishes ROS2 PointCloud2 messages
- **Integration**: Inherits from ROS2SensorComponentBase with TickBasedSource

## Key Components

### SimpleLidar Class (`Code/Source/Clients/SimpleLidar.h:96`)
Main component implementing the LIDAR sensor functionality.

### PendingFrames Struct (`Code/Source/Clients/SimpleLidar.h:19`)
Manages synchronized capture of color and depth frames from multiple cameras.

## Development Workflow

### Build Commands
```bash
# From project root
cmake --build build --target SimpleLidarSensor
```

### Testing
```bash
# Run unit tests
ctest --test-dir build -R SimpleLidarSensor
```

### Key Files
- **Main Implementation**: `Code/Source/Clients/SimpleLidar.cpp`
- **Header**: `Code/Source/Clients/SimpleLidar.h`
- **Module Interface**: `Code/Source/SimpleLidarSensorModuleInterface.h`
- **Type IDs**: `Code/Include/SimpleLidarSensor/SimpleLidarSensorTypeIds.h`
- **Bus Interface**: `Code/Include/SimpleLidarSensor/SimpleLidarSensorBus.h`

### Dependencies
- O3DE Atom renderer (RPI.Public)
- OpenCV
- ROS2 (sensor_msgs, rclcpp)
- ROS2 sensor base classes

### Common Tasks

#### Modifying Camera Configuration
- Update `ViewCount` constant for different camera arrangements
- Adjust `HorizontalFOV` calculation accordingly
- Modify camera transforms in `m_cameraToLidarCoordinate`

#### Point Cloud Processing
- Main processing happens in `PublishPointCloud()` method
- Frame synchronization handled by `PendingFrames::IsComplete()`

#### ROS2 Integration
- Publisher setup: `m_pointCloudPublisher`
- Message publishing via inherited ROS2SensorComponentBase

### Testing Files
- **Editor Tests**: `Code/Tests/Tools/SimpleLidarSensorEditorTest.cpp`
- **Client Tests**: `Code/Tests/Clients/SimpleLidarSensorTest.cpp`

## Notes
- Uses Atom renderer for camera captures
- Implements Rule of 5 for PendingFrames (proper copy/move semantics)
- Thread-safe frame handling with mutex protection
- Camera matrix shared across all views in the rig