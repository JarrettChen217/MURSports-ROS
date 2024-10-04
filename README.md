# MURSports-ROS
Welcome to the MURSports Ros team Repo. This is where the source files for the ROS team will be stored.

## File path

* Robots Playground.usd
  * the Nvidia Issac Project USD file.

---

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

---

## Python Code Controller/Subscriber

> Typically, the Python libraries that directly link with ROS2, such as `rclpy` and many `msg` types, are bundled with ROS2, so there is no need for special installation. Therefore, some dependencies are ~~crossed out~~. *However, it is recommended to download ROS2-related extensions in your IDE, such as VSCode, to enable auto-completion features.*

> [!tip]
>
> Regarding ROS2 topics, we can check them using the `ros2 topic list` command.
>
> Don't forget to start the Isaac simulation! (Click the play button)

---

### 1. `subscribe_cam.py`

Receives image frames from the camera topic and displays the images using OpenCV.

#### Dependencies:
- **Related ROS2 Topic**: `/front_stereo_camera/left_rgb/image_raw`
- **Required Python Libraries**:
  - ~~`rclpy`: Python client library for ROS2 nodes~~
  - `cv_bridge`: Converts ROS image messages to OpenCV format
  - `opencv-python`: Used for image processing and display

##### Install Additional Libraries:

cv_bridge: usually be `sudo apt-get install ros-humble-cv-bridge`

```bash
sudo apt-get install ros-<your_ros_distro>-cv-bridge
```

opencv-python: 

```bash
pip install opencv-python
```

(Replace `<your_ros_distro>` with your ROS2 distribution, such as `foxy` or `galactic`.)

#### Run Command:

```bash
python3 subscribe_cam.py
```

Here, we focus on the `image_callback(self, msg)` function, which is invoked every time a new image message is published by ROS.

* By using the cv2 library to display the images, we can obtain a coherent video stream.

---

### 2. `robot_controller.py`

Controls the robot's movement via keyboard input and publishes velocity commands to the `/cmd_vel` topic.

#### Dependencies:
- **Related ROS2 Topic**: `/cmd_vel`
- **Required Python Libraries**:
  - ~~`rclpy`: Python client library for ROS2 nodes~~
  - ~~`geometry_msgs`: ROS2 message type used for publishing velocity commands~~
  - `pynput`: Used for listening to keyboard inputs

##### Install Additional Libraries:
```bash
pip install pynput
```

#### Run Command:
```bash
python3 robot_controller.py
```

We can then use "`w`, `a`, `s`, `d`" to control the cart to move back and forth, we allow forward or backward to be pressed at the same time as left or right to make small directional corrections. Use `Esc` to stop the input (but the node will still be active, please stop the node with `Ctrl+C`).

> [!note]
>
> Currently there is a delay in the steering movement of the cart, which needs to be debugged.

---

### 3. `read_odem.py`

Reads odometry data from the `/odom` topic and processes or displays the data.

#### Dependencies:
- **Related ROS2 Topic**: `/odom`
- **Required Python Libraries**:
  - ~~`rclpy`: Python client library for ROS2 nodes~~
  - ~~`nav_msgs`: ROS2 message type used for receiving odometry data~~

##### Install Additional Libraries:
[None]

#### Run Command:
```bash
python3 read_odem.py
```

Subsequent parameters such as linear speed, angular speed will be logged in the terminal.

> [!tip]
>
> Try adding more parameters if needed

---

### 4. `robot_controller_xbox.py`

#### Dependencies:

- **Related ROS2 Topic**: `/cmd_vel`
- **Required Python Libraries**:
  - `pygame`: Used for listening to joysticks inputs

##### Install Additional Libraries:

```bash
pip install pygame
```

#### Run Command :

```bash
python3 robot_controller_xbox.py
```

> The code will automatically detect the joystick connected to the computer and select the first joystick for connection <u>(if no joystick is found, it will fail and exit directly!)</u>

Then, we can use the joystick to control the movement of the car.

> (Referring to the key settings of racing games, you can also modify it according to the axis [description of pygame](https://www.pygame.org/docs/ref/joystick.html).)

Current key settings:

* <u>Left Joystick_x</u> (axis 0): **Left** and **Right** movement (steer)
* <u>Left Trigger</u> (axis 4): **Forward** movement (throttle)
* <u>Right Trigger</u> (axis 5): **Backward** movement (brake)
* <u>No Input</u>: **Stop**

> [!note]
>
> In order to prevent joystick drift caused by handle aging, a "dead zone" of `0.1` has been set.

#### Parameter adjustment

##### Max speed

Max speed adjustable, through:

* `self.max_linear`
* `self.max_angular`

##### Publish rate

> [!note]
>
> May affect the resource usage or lead to lagging!

Publish rate adjustable, through:

* `self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz`

#### Closing the controller

Need to take extra care of shutting this node down!

Call `shutdown()` function every time! In order to quit `pygame`.



# application file

try add usd file into the stage by python code
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/Documentation/Isaac-Sim-Docs_2021.2.1/app_isaacsim/app_isaacsim/manual_standalone_python.html