import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# import sys
# sys.path.append("E:\Octo_demo_collection_UR5\RTDE_Python_Client_Library")
# from RTDE_Python_Client_Library import rtde
# # from RTDE_Python_Client_Library import rt


# 机器人的IP地址
ROBOT_HOST = '192.168.4.100'
# 与RTDE使用的端口号
ROBOT_PORT = 50002

# 创建一个RTDE对象
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
# 连接到机器人
con.connect()

# 检查连接是否成功
if not con.isConnected():
    print("无法连接到机器人")
    exit()

# 获取并打印一些信息，例如机器人的当前状态
print("机器人当前状态:", con.receive())

# 断开连接
con.disconnect()
