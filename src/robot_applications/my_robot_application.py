
from omni.isaac.kit import SimulationApp

# 启动 Isaac Sim 仿真应用 before importing other libiaries.
simulation_app = SimulationApp({
    "headless": False,  # 如果不需要 GUI，可以设置为 True
})

import asyncio
# import omni
import omni.usd
# import rclpy
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.articulations import Articulation
from pynput import keyboard  # 导入 pynput 库以监听键盘输入



# Enable the ros2_bridge manually.
enable_extension("omni.isaac.ros2_bridge")
# ext_manager = omni.kit.app.get_app().get_extension_manager()
# ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)


# 创建仿真世界
world = World()

usd_full_path = "/home/haochen/Github_Repos/MURSports-ROS/Robots_Playground.usd"
add_reference_to_stage(usd_path=usd_full_path, prim_path="/World")

world.reset()

# keyListener to exit loop================================================================
running = True
robot_obj = None
reset_flag = False

def on_press(key):
    global running, robot_obj, reset_flag
    try:
        if key == keyboard.Key.esc:  # 如果按下 "Esc" 键
            print("Esc pressed, exiting the loop.")
            running = False  # 终止循环
        elif hasattr(key, 'char') and key.char == 'g':
            robot_obj = get_robot_object()
        elif hasattr(key, 'char') and key.char == 'r':
            get_robot_position(robot_obj)
        elif hasattr(key, 'char') and key.char == 'f':
            reset_flag = True
    except AttributeError:
        pass

# 创建键盘监听器，并启动监听
listener = keyboard.Listener(on_press=on_press)
listener.start()
# =========================================================================================

def get_robot_object():
    robot_chassis_path = "/World/Jackal/base_link"
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(robot_chassis_path)
    obj = None
    if prim.IsValid():
        print("Object found!")
        obj = Articulation(prim_path=robot_chassis_path, name="Jackal_chassis")
        if not obj:
            print("articulation convension failed")
        else:
            print(prim)
    else:
        print("Object not found.")
    return obj

# 定义一个异步函数以定期获取机器人的位置
def get_robot_position(robot):
    if robot:
        # 获取机器人的位姿
        position, orientation = robot.get_world_pose()
        linear_velocity = robot.get_linear_velocity()

        # 打印机器人的位置、朝向和线速度
        print(f"Robot position: {position}")
        print(f"Robot orientation: {orientation}") # not arcurate.
        print(f"Robot linear velocity: {linear_velocity}")
    else:
        print("robot object is null!")

# reset the simulation world
def reset_world():
    print("Resetting world...")
    world.stop()  # 暂停仿真
    world.reset()  # 重置仿真
    world.play()   # 重新开始仿真

# 无限循环
try:
    while running:
        # 控制物理和渲染步进，同步运行
        world.step(render=True)  # 执行一个物理步和一个渲染步
        if reset_flag:
            reset_world()
            reset_flag = False  # 重置标志清除
finally:
    # stop the keyboard listener
    listener.stop()
    # 关闭 Isaac Sim
    simulation_app.close()
