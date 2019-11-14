# Basic example source code to measure round trip time from ROS2 -> PX4 -> ROS2

This done by sending a ROS2 msg to the /DebugKeyValue_PubSubTopic with a timestamp value

This ROS2 msg will be transfered to PX4 using px4_ros_com

PX4 on a FMU will relay the msgs from the DebugKeyValue uORB topic to the Airspeed topic uORB

The px4 micrortps_client sends back the data to the px4_ros_com and px4_ros_com sends the data to the ROS2 domain

The ROS2 listener will listen to msgs on the /Airspeed_PubSubTopic and prints out the relative time it took


## UML diagram


```mermaid
sequenceDiagram


loop Every 500ms
ros2_px4_profiler ->> micrortps_agent: DebugKeyValue
micrortps_agent ->> micrortps_client: DebugKeyValue
micrortps_client->> uorb_relay: DebugKeyValue
uorb_relay ->> micrortps_client: AirSpeed
micrortps_client->> micrortps_agent : AirSpeed
micrortps_agent ->> ros2_px4_profiler : AirSpeed
ros2_px4_profiler ->> ros2_px4_profiler : Print(timing)
end
```

