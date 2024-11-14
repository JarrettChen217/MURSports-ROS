from omni.isaac.kit import SimulationApp

# 启动 Isaac Sim 仿真应用 before importing other libiaries.
simulation_app = SimulationApp({
    "headless": False,  # 如果不需要 GUI，可以设置为 True
})

import asyncio
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.articulations import Articulation
from pynput import keyboard  # 导入 pynput 库以监听键盘输入

# Enable the ros2_bridge manually.
enable_extension("omni.isaac.ros2_bridge")

# 创建仿真世界
world = World()

# 加载 USD 文件
usd_full_path = "/home/haochen/Github_Repos/MURSports-ROS/Robots_Playground.usd"
add_reference_to_stage(usd_path=usd_full_path, prim_path="/World")

# 重置世界以确保物理句柄正确传播
world.reset()

# 键盘监听器逻辑变量
running = True
robot_obj = None
reset_flag = False

# 键盘事件处理函数
def on_press(key):
    global running, robot_obj, reset_flag
    try:
        if key == keyboard.Key.esc:
            print("Esc pressed, exiting the loop.")
            running = False
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

# 获取机器人对象的函数
def get_robot_object():
    robot_chassis_path = "/World/Jackal/base_link"
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(robot_chassis_path)
    obj = None
    if prim.IsValid():
        print("Object found!")
        obj = Articulation(prim_path=robot_chassis_path, name="Jackal_chassis")
        if not obj:
            print("Articulation conversion failed")
        else:
            print(prim)
    else:
        print("Object not found.")
    return obj

# 获取机器人的位置
def get_robot_position(robot):
    if robot:
        # 获取机器人的位姿
        position, orientation = robot.get_world_pose()
        linear_velocity = robot.get_linear_velocity()

        # 打印机器人的位置、朝向和线速度
        print(f"Robot position: {position}")
        print(f"Robot orientation: {orientation}")  # not accurate
        print(f"Robot linear velocity: {linear_velocity}")
    else:
        print("Robot object is null!")

# 重置仿真世界
def reset_world():
    print("Resetting world...")
    world.stop()  # 暂停仿真
    world.reset()  # 重置仿真
    world.play()   # 重新开始仿真

# 无限循环以维持仿真
try:
    while running:
        # 控制物理和渲染步进，同步运行
        world.step(render=True)  # 执行一个物理步和一个渲染步
        if reset_flag:
            reset_world()
            reset_flag = False  # 重置标志清除
finally:
    # 停止键盘监听器
    listener.stop()
    # 关闭 Isaac Sim
    simulation_app.close()
