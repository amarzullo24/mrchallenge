# Welcome
Welcome to the MR Course challenge 2021/22: You Can't See Me
In this challenge you are called to create an AI for the [Da Vinci Robot](https://www.intuitive.com/en-us/products-and-services/da-vinci/systems)'s endoscope.
In particular, the endoscope should automatically follow the movements of the patient side manipulator (PSM).

## simulated environment
The challenge arena will be a simulated environment in which the robot is placed.
The [ambf](https://github.com/WPI-AIM/ambf/tree/restructure) simulation environment will be used (restructure branch).

as you will learn from the [wiki](https://github.com/WPI-AIM/ambf/wiki), the sistem offers several funtionalities to interact with the simulated objects in the scene. All the message exchange part is handled by the sistem through a propoer Python Client. All you need to do is to use the provided information to give birth to the endoscope's brain!

Let's start!

# Setting up the system
This repository contains all the codes and packages to setting up the system. The code has been tested for **Ubuntu 18.04**

## 1. Clone the repository
Clone the repository on your computer and follow the instructions:

## 2. Install ambf:

1. On Linux machines you may need to install the following packages:
````
sudo apt install libasound2-dev libgl1-mesa-dev xorg-dev
````

2. Then, you need to install ambf:
```
cd ambf && mkdir build
git submodule update --init --recursive
cd build
cmake ..
make
```
If the building ends with no errors you have done!

# Start the system
Once installed, *ambf* needs to be started. You can start the simulation by typing:
````
cd path_to_ambf/bin/lin-x86_64
./ambf_simulator -l 5,6
````
Two windows should appear: the first one shows you a "third person" view of the scene. The second one shows you a "first person" view of the endoscope (it shows the images from endoscope's camera)

## 1. Moving the PSM
Once your system is started properly, in order to "test" the code you are going to write, you may need to automatically move the PSM of the DaVinci.
I wrote a simple Python Script for doing that for you. You can find it under the "challenge" folder, named as "move_tool.py".
Run it as
````
python move_tool.py
````
You should see the PSM performing a linear trajectory along an axis.

***Notice*** that this is the exact same script I am going to run during to create the leaderboard score! (of course I will change the target trajectory)

## 2. Start coding
Two more useful files: 
* the "evaluator.py" contains the code used to compute the evaluation metrics. It is also a nice reference to better understand how the *ambf Pyhton Client* works in practice and how you can handle and process information coming from the ROS topics.
* Finally, the "brain.py" script is the core of the challenge. It is supposed to contain the actual code you are going to write. It is already precompiled such that you can automatically log the information computed by the evaluator in real-time.

# Final remarks
1. I will constatly update this repository as the challenge goes on. It is reasonalbe that bugs, improvements of the scene, modifications will be identified/needed by you. To stay updated, one of the best way is to [fork](https://docs.github.com/en/get-started/quickstart/fork-a-repo) the github repository, so you will have your own private project and you can integrate changements from this repository without affecting the code, like [here](https://levelup.gitconnected.com/how-to-update-fork-repo-from-original-repo-b853387dd471)
3. You are not allowed to use information coming from the PSM (e.g. it's position)

Useful refs:
* [AMBF](https://github.com/WPI-AIM/ambf/tree/restructure)
* [ROS](http://wiki.ros.org/ROS/Tutorials)
* [Python Tutorial](https://www.w3schools.com/python/)
* [Git commands in a nutshell](http://rogerdudler.github.io/git-guide/)


