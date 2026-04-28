from env import RealUR5
from robot import UR5ArmController
from utilities import RealsenseCamera, GamepadController

import pygame
import cv2

def user_control_demo():

    env = RealUR5()
    env.reset()
    gamepad = GamepadController()
    gamepad.start()


    try:
        while True:
            state = gamepad.read()

            current_action_rotvec = env.robot.info.getActualTCPPose()
            current_gripper_position = env.robot.gripper.last_send_value
            action_rotvec = gamepad.map_gamepad_to_action(state, current_action_rotvec, current_gripper_position)

            gamepad.handle_button_down(state)
            # 获取任务状态
            cup_picked = gamepad.task_status["cup_picked"]
            cup_reset = gamepad.task_status["cup_reset"]
            mug_picked = gamepad.task_status["mug_picked"]
            mug_reset = gamepad.task_status["mug_reset"]

            # 将处理后的控制参数传递给环境
            action_euler = env.read_control_parameter(action_rotvec, cup_picked, cup_reset, mug_picked, mug_reset)
            # print(cup_picked, cup_reset, mug_picked, mug_reset)
            # 在环境中执行动作
            obs, reward, terminated, truncated, info = env.step(action_euler)
            print(info)
            # 显示接收到的观测图像
            if "image_primary" in obs:
                image_bgr = cv2.cvtColor(obs["image_primary"], cv2.COLOR_RGB2BGR)
                cv2.imshow("Observation", image_bgr)
                cv2.waitKey(1)

                # cv2.imshow("Observation", obs["image_primary"])

            # # 按下 'q' 键退出循环
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            # 添加一个小的延时来避免过度占用CPU
            pygame.time.wait(50)

    except KeyboardInterrupt:
        gamepad.stop()
        env.close()
        print("Control stopped.")

if __name__ == "__main__":
    user_control_demo()