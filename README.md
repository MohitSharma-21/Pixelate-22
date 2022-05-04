# Pixelate '22 Sample Arena

## Installation Guidelines

Before installing this arena, you need to download certain modules on which it is dependent. We **strongly** recommend using a distribution of **Linux** as your operating system for this event. Windows installations tend to be a hassle and require, in some instances, quite a bit of time and effort to complete.

0. Although not compulsory, we strongly recommend creating a virtual environment specific to this project. This will help in package management and for decluttering your workspace. A simple way to create a virtual environment is as follows:

   ```bash
   python3 -m venv <Env_Name>
   ```

   Activation and deactivation of your virtual environment, will be done as specified [here](https://docs.python.org/3/library/venv.html). Scroll down to the table where the activation method for various operating systems is provided. Deactivation, in most cases, can be done by simply typing deactivate while being in in the virtual environment.

1. Once you activate your virtual environment, you will have to install the various dependencies of this project. We have simplified this process for you. Just follow the following steps:

   - Download/Clone this repository on to your local machine.
   - Navigate to the root folder of this repository through your terminal.
   - Execute the following command in your terminal.

     ```bash
     pip install -e pixelate_arena
     ```

   - To check whether the installation has been successful, you can refer to our guide/cheatsheet to know how to build the gym in your own python script as well as use the utility functions.

In case there are problems with the PyBullet installation, you can refer to this [guide](https://github.com/Robotics-Club-IIT-BHU/Robo-Summer-Camp-20/blob/master/Part1/Subpart%201/README.md).

## Using the Arena

0. You will have to import the package pix_arena, which will be available only if you've completed step 1 in the Installation Guidelines. The arena can be initialized by using:

```python
env = gym.make("pixelate_arena-v0")
```

1. Then, you will have to create the working loop, as is normally done in pybullet (using `stepSimulation()`).

2. The functions of the environment, available to you for various purposes, are as follows. Please go through the examples of the functions in the examples folder, if you wish to know their arguments and/or return values.

   - `env.camera_feed()`  
      This will return an RGB image of the arena as if a camera was placed on top of the arena.
   - `env.remove_car()`  
      This will be used to remove the car from the arena, in case you want to have a good look at it.
   - `env.respawn_car()`  
      This will be used to respawn the car into the arena, **only after removing it**.
   - `env.move_husky()`  
      This will be used to give the motor velocity to each wheel individually of the car.
   - `env.unlock_antidotes()`  
      This will be used to reveal the locations of antidotes as specified in the PS.
   - `env.reset()`
     This will reset the whole arena. This function cannot be used for your final submission.

3. You can refer the file **function_info.py** to see the documentation of the different functions and **aruco_test.py** to see the detection of aruco marker.

## Solution
After activating your virtual environment

     cd solution
     
     python BOT.py
     
