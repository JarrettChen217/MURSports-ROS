
from omni.isaac.kit import SimulationApp
import rclpy

# 指定要加载的扩展列表
extensions_to_load = [
    "omni.isaac.core",
    "omni.isaac.ros2_bridge",
    "omni.isaac.core_nodes",
    "omni.graph.bundle.action",
    # 其他您可能需要的扩展
]

# 启动 Isaac Sim 仿真应用，并指定要加载的扩展
simulation_app = SimulationApp({
    "headless": False,  # 如果不需要 GUI，可以设置为 True
    "exts": extensions_to_load
})
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import asyncio
from pynput import keyboard  # 导入 pynput 库以监听键盘输入
import omni

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)


# 创建仿真世界
world = World()

usd_full_path = "/home/haochen/Github_Repos/MURSports-ROS/Robots_Playground.usd"
add_reference_to_stage(usd_path=usd_full_path, prim_path="/World")

# 重置世界以确保物理句柄正确传播
world.reset()

rclpy.init()



# keyListener to exit loop================================================================
running = True

# 定义一个回调函数，当按下 "Esc" 键时，将 `running` 设置为 False，终止循环
def on_press(key):
    global running
    try:
        if key == keyboard.Key.esc:  # 如果按下 "Esc" 键
            print("Esc pressed, exiting the loop.")
            running = False  # 终止循环
    except AttributeError:
        pass

# 创建键盘监听器，并启动监听
listener = keyboard.Listener(on_press=on_press)
listener.start()
# =========================================================================================

# 获取机器人对象
robot = world.scene.get_object("Jackal")

# 确保机器人对象存在
if not robot:
    print("Error: Robot 'my_robot' not found in the scene.")
    simulation_app.close()
    exit(1)

# 定义一个异步函数以定期获取机器人的位置
async def get_robot_position():
    while simulation_app.is_running():
        # 获取机器人的位姿
        position, orientation = robot.get_world_pose()
        linear_velocity = robot.get_linear_velocity()

        # 打印机器人的位置、朝向和线速度
        print(f"Robot position: {position}")
        print(f"Robot orientation: {orientation}")
        print(f"Robot linear velocity: {linear_velocity}")

        # 等待下一个物理步
        await asyncio.sleep(0.1)  # 根据需要调整间隔时间


# 主函数，运行仿真和获取机器人位置
async def main():
    # 启动获取位置的任务
    position_task = asyncio.create_task(get_robot_position())

    # 无限循环
    try:
        while running:
            # 控制物理和渲染步进，同步运行
            world.step(render=True)  # 执行一个物理步和一个渲染步
            await asyncio.sleep(0)  # 让出控制权
    finally:
        # 确保监听器在程序退出时停止
        listener.stop()
        # 关闭 ROS2
        rclpy.shutdown()
        # 关闭 Isaac Sim
        simulation_app.close()

    # 等待位置任务完成
    await position_task
# 运行主函数
asyncio.run(main())

# 关闭 Isaac Sim 仿真应用
simulation_app.close()
