# Using the Pose Estimation node

After you have built the node in your `catkin_ws`, start `roscore` in a terminal. Then in a new terminal:
```bash
cd catkin_ws

source devel/setup.bash

roslaunch pose_estimation_6d pose_estimation.launch recording_file:=/{absolute path to mkv recording} cad_path:=/{absolute path to cad model}
```

The above launch is only if you are using a pre-recorded video. If you want to use the kinect live stream, you can omit `recording_file`. However `cad_path` is necessary.

Please have a look at the [launch file](../launch/pose_estimation.launch) for more details on what parameters you can modify.


