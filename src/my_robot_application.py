# 导入必要的模块
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.objects import Articulation
import numpy as np
import asyncio

# 启动 Isaac Sim 仿真应用
simulation_app = SimulationApp({"headless": False})

# 创建仿真世界
world = World()

# 加载 USD 文件
usd_file_path = "/home/haochen/Github_Repos/MURSports-ROS/Robots_Playground.usd"
world.scene.add(usd_file_path)

# 重置世界以确保物理句柄正确传播
world.reset()

# 获取机器人对象（假设机器人名称为 'my_robot'）
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

# 创建任务以获取机器人位置
async def main():
    # 启动获取位置的任务
    position_task = asyncio.create_task(get_robot_position())

    # 运行仿真主循环
    while simulation_app.is_running():
        world.step(render=True)  # 推进一个物理步和一个渲染步
        await asyncio.sleep(0)  # 让出控制权

    # 等待位置任务完成
    await position_task

# 运行主函数
asyncio.run(main())

# 关闭 Isaac Sim 仿真应用
simulation_app.close()
