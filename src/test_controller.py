import pygame

# 初始化pygame和控制器
pygame.init()
pygame.joystick.init()

# 检查系统中的控制器数量
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick detected.")
else:
    # 使用第一个控制器
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected controller: {joystick.get_name()}")

# 主循环
try:
    while True:
        # 处理pygame事件队列
        for event in pygame.event.get():
            # 处理控制器的按钮按下事件
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

            # 处理控制器的按钮释放事件
            if event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} released")

            # 处理控制器的轴移动（例如摇杆）的事件
            # if event.type == pygame.JOYAXISMOTION:
                # axis = event.axis
                # value = event.value
                # print(f"Axis {axis} moved to {value}")
            right_trigger = joystick.get_axis(5)  # RT button
            right_trigger += 1
            if(right_trigger > 0.01):
                print(f"RT moved to {right_trigger}")
            left_trigger = joystick.get_axis(2)
            left_trigger += 1
            if(left_trigger > 0.01):
                print(f"LT moved to {left_trigger}")

            left_stick_x = joystick.get_axis(0)
            if(left_stick_x < -0.1):
                print(f"turn left {-left_stick_x}")
            elif(left_stick_x > 0.1):
                print(f"turn right {left_stick_x}")

except KeyboardInterrupt:
    print("Program terminated.")

# 关闭pygame
pygame.quit()