import gym
import pixelate_arena
import time

if __name__ == '__main__':
    env = gym.make("pixelate_arena-v0" , sim=False)
    time.sleep(1)

    print("Documentation for functions to interact with the arena")
    
    print("move_husky function :")
    print(env.move_husky.__doc__)

    print("camera_feed function :")
    print(env.camera_feed.__doc__)

    print("unlock_antidotes function :")
    print(env.unlock_antidotes.__doc__)

    print("remove_car function :")
    print(env.remove_car.__doc__)

    print("respawn_car function :")
    print(env.respawn_car.__doc__)

    print("reset_arena_function :")
    print(env.reset_arena.__doc__)


