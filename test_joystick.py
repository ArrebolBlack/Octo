import pygame

# 初始化 Pygame
pygame.init()

# 初始化 Pygame 的 joystick 模块
pygame.joystick.init()

# 获取连接的手柄数量
joystick_count = pygame.joystick.get_count()
print(f"Connected joystick count: {joystick_count}")

# 检查是否至少有一个手柄连接
if joystick_count == 0:
    print("No joysticks were found.")
else:
    # 初始化第一个手柄 (编号为 0)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick {joystick.get_name()} is initialized.")
