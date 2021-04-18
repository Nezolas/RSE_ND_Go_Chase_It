# Udacity Robotic Software Engineer Nanodegree Program
## Go Chase It! (Project 2)
_All readme documentations and informations are taken from Udacity Project Submission Page_
## Project Description
Summary of Tasks
In this project, two ROS packages are created inside the `catkin_ws/src:` the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:



`drive_bot:`

* The my_robot ROS package created to hold your robot, the white ball, and the world.
* Differential drive robot designed with the Unified Robot Description Format. Two sensors added to your robot: a _lidar_ and a _camera_. Gazebo plugins added for robot’s differential drive, lidar, and camera.
* House your robot inside the world you built in the Build My World project.
* White-colored ball added to Gazebo world and saved a new copy of this world.
* The world.launch file launch world with the white-colored ball and the robot.

`ball_chaser:`
Ball_chaser ROS package created to hold C++ nodes.
Drive_botC++ node writed that will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
Process_image writed C++ node that reads robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, node should request a service via a client to drive the robot towards it.
The ball_chaser.launch should run both the drive_bot and the process_image nodes.

### File Structure
```
.Project2                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── <yourworld>.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                            
```
## Run The Project
* **Build Package**
Now that you’ve included specific instructions for your `process_image.cpp` code in `CMakeLists.txt`, compile it with:
```
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

* **1- Launch the robot inside your world**

This can be done by launching the `world.launch` file:
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

* **2- Run drive_bot and process_image**

This can be done by executing `ball_chaser.launch`:

$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch

* **3- Visualize**

To visualize the robot’s camera images, you can subscribe to camera RGB image topic from RViz. Or you can run the rqt_image_view node:
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view  
```
Now place the white ball at different positions in front of the robot and see if the robot is capable of chasing the ball!
