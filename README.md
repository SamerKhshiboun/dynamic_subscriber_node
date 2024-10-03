# Dynamic Subscriber Node

This package contains a dynamic ROS 2 subscriber node that calculates the frequency (Hz) of incoming messages for any given topic. The topic name and message type can be specified via command-line parameters when launching the node.

## Installation and Setup

### Prerequisites

Ensure that you have a working ROS 2 environment installed. The package is compatible with ROS 2 distributions such as Humble, Iron, or Jazzy. The package depends on the following ROS 2 packages:
- `rclcpp`
- `sensor_msgs`
- `std_msgs`

### Building the Package

1. Navigate to your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    ```

2. Clone the repository:
    ```bash
    git clone https://github.com/SamerKhshiboun/dynamic_subscriber_node
    ```

3. Build the package using `colcon`:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

4. Source the workspace after building:
    ```bash
    source install/setup.bash
    ```

## Usage

The node subscribes to a specified topic and calculates the frequency of incoming messages. You can provide the topic name and message type through command-line arguments.

### Running the Node

You can run the node using the `ros2 run` command, passing the `topic_name` and `topic_type` parameters.

#### General Syntax:
```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=<topic_name> -p topic_type:=<topic_type>
```

- `topic_name`: The ROS 2 topic to which the node should subscribe.
- `topic_type`: The message type of the topic (e.g., `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`).

### Examples

Here are some examples of running the node with different topics and message types:

#### 1. `/camera/camera/depth/image_rect_raw` of type `Image`

For subscribing to the depth image topic:

```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=/camera/camera/depth/image_rect_raw -p topic_type:=sensor_msgs/msg/Image
```

#### 2. `/camera/camera/depth/image_rect_raw/compressedDepth` of type `CompressedImage`

For subscribing to the compressed depth image topic:

```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=/camera/camera/depth/image_rect_raw/compressedDepth -p topic_type:=sensor_msgs/msg/CompressedImage
```

#### 3. `/camera/camera/color/image_raw` of type `Image`

For subscribing to the raw color image topic:

```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=/camera/camera/color/image_raw -p topic_type:=sensor_msgs/msg/Image
```

#### 4. `/camera/camera/color/image_raw/compressed` of type `CompressedImage`

For subscribing to the compressed color image topic:

```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=/camera/camera/color/image_raw/compressed -p topic_type:=sensor_msgs/msg/CompressedImage
```

### Best-Effort QoS Example

If you want to run the node using Best-Effort Quality of Service (QoS), you can pass the `qos_reliability` parameter:

```bash
ros2 run dynamic_subscriber_node dynamic_subscriber_node --ros-args -p topic_name:=/camera/camera/color/image_raw -p topic_type:=sensor_msgs/msg/Image -p qos_reliability:=best_effort
```

### Additional Parameters

- `qos_reliability`: Specify the reliability QoS policy. Valid values are `reliable` (default) and `best_effort`.

## Modifying the Code

If you need to handle additional message types or further customize the node, you can edit the source file (`src/dynamic_subscriber_node.cpp`). Ensure that any additional message types are linked in the `CMakeLists.txt` and `package.xml`.

