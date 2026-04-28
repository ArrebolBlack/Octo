from octo_ur5.real.real_ur5_env import RealUR5
from octo_ur5.real.utilities import RealsenseCamera, GamepadController

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
            cup_picked = gamepad.task_status["cup_picked"]
            cup_reset = gamepad.task_status["cup_reset"]
            mug_picked = gamepad.task_status["mug_picked"]
            mug_reset = gamepad.task_status["mug_reset"]

            action_euler = env.read_control_parameter(action_rotvec, cup_picked, cup_reset, mug_picked, mug_reset)
            obs, reward, terminated, truncated, info = env.step(action_euler)
            print(info)

            if "image_primary" in obs:
                image_bgr = cv2.cvtColor(obs["image_primary"], cv2.COLOR_RGB2BGR)
                cv2.imshow("Observation", image_bgr)
                cv2.waitKey(1)

            pygame.time.wait(50)

    except KeyboardInterrupt:
        gamepad.stop()
        env.close()
        print("Control stopped.")


if __name__ == "__main__":
    user_control_demo()
