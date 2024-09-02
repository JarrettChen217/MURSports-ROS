# MURSports-ROS
Welcome to the MURSports Ros team Repo. This is where the source files for the ROS team will be stored.

## File path

* Robots Playground.usd
  * the Nvidia Issac Project USD file.



## How to perform the Nvidia Issac

### Setup/Check the environment

> Defaultly have the "Nvidia Issac" and "ROS2" Downloaded, activate the "ROS2 Bridge Extension".

1. <u>Open</u> the Nvidia Issac and the USD file ready. 
2. <u>Click</u> Play Button on the left hand side.
3. <u>Open</u> the Terminal, and input command `ros2 topic list`, to check the ros bridge is ready to find the *topics*

if the topic is showned simillar as below, you are ready to make the robot move!



### Make the robot moving by terminal

> [!caution]
>
> Make sure `/cmd_vel` is in the topic list.

1. Continue on the terminal/New a terminal, input command 

   ```cmd
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
   ```

   1. This command input two `Vector3` arguments into the `/cmd_vel` topic's `geometry_msgs/Twist` node. 
      1. ==linear== argument takes 'x' as the robot's linear speed.
      2. ==angular== argument takes 'Z' as the robot's angular speed.

> [!note]
> The nodes will continue performing the action that you inputed, unless you type in another command to change it's node.
>
> * Stop robot movement: 
>
>   ```cmd
>   ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
>   ```



### Make the robot moving by keyboard

> This requires that we install the `teleop_twist_keyboard` library for ros, which we can get by typing the following command.
>
> ```cmd
> sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
> ```

> [!caution]
>
> Again! Make sure `/cmd_vel` is in the topic list.

To the terminal, type

```cmd
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

and it will show an detailed key map to control the `/cmd_vel` topic, which will move our robot.
