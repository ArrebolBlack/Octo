import rtde_control
import rtde_receive
import time

def connect_robot(robot_ip):
    try:
        # 尝试创建控制和接收接口的实例
        rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        print(f"Successfully connected to the robot at {robot_ip}")
        return rtde_c, rtde_r
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")
        return None, None

def test_robot_connection(robot_ip):
    # 连接机器人
    rtde_c, rtde_r = connect_robot(robot_ip)

    if rtde_c is None or rtde_r is None:
        print("Exiting test due to connection failure.")
        return

    try:
        # 获取机器人的当前TCP位置
        tcp_pose = rtde_r.getActualTCPPose()
        print(f"Current TCP pose: {tcp_pose}")

        # 发送一个简单的运动指令 (例如: 往上移动 10cm)
        target_pose = tcp_pose.copy()
        target_pose[2] += 0.05  # z轴上移动10cm

        print("Moving robot up by 10cm...")
        rtde_c.moveL(target_pose, speed=0.5, acceleration=0.2)

        # 等待运动完成
        time.sleep(2)

        # 获取新的TCP位置
        new_tcp_pose = rtde_r.getActualTCPPose()
        print(f"New TCP pose after movement: {new_tcp_pose}")

    except Exception as e:
        print(f"An error occurred during the test: {e}")
    finally:
        # 确保在退出前停止RTDE控制
        if rtde_c is not None:
            rtde_c.stopScript()
        print("Test completed.")

if __name__ == "__main__":
    robot_ip = "192.168.4.100"  # 替换为你机器人的实际IP地址
    test_robot_connection(robot_ip)
