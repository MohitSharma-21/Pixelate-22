import gym
import time
import pixelate_arena
import pybullet as p
import pybullet_data
import cv2
import os

if __name__ == "__main__":
    parent_path = os.path.dirname(os.getcwd())
    os.chdir(parent_path)
    env = gym.make("pixelate_arena-v0")
    time.sleep(0.5)
    env.remove_car()
    time.sleep(0.5)
    env.respawn_car()
    time.sleep(0.5)