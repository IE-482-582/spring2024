# Followbot
This project comes from Chapter 12 of the textbook, but modified significantly to work with our Husky.

The goal is to direct a Husky to drive around a course/track, characterized by a line of a certain color.

---
## The `followbot` ROS Package
First, we need to create a ROS "package" for this project.  

We will assume that you have cloned the course GitHub respository to `~/Projects/IE-482-582/spring2024`, and that your ROS packages will be installed in `~/catkin_ws/src`.

1.  Create the `followbot` package:
    ```
    cd catkin_ws/src
    catkin_create_pkg followbot
    ```

2.  Copy files from the GitHub repo to the catkin workspace:
    ```
    cp -r ~/Projects/IE-482-582/spring2024/Textbook/chapter_12_followbot/* ~/catkin_ws/src/followbot/
    ```

3.  Re-build the catkin workspace
    ```
    cd ~/catkin_ws
    catkin_make
    ```

--- 

Now, let's get an idea of the operating conditions.

### Terminal 1: Start Husky
```
roslaunch husky_gazebo husky_playpen.launch
```

### Terminal 2: View camera feed
```
rosrun followbot view_camera.py
```

### Terminal 3: Start keyboard teleop
This will allow us to move around and watch the camera.  Next, we'll direct the Husky to drive autonomously.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### NOTES
- If the video feed shows a giant black area across the bottom of the image, it's likely that the camera is being blocked by the Husky's frame.  These environment variables should fix that:
    ```
    export HUSKY_REALSENSE_ENABLED=1
    export HUSKY_REALSENSE_XYZ="0.35 0 0.01"
    ```

- Let's see how fast our camera feed is operating:
    ```
    rostopic list
    rostopic hz /realsense/color/image_raw
    ```

- What information is being passed by this topic?
    ```
    rostopic echo /realsense/color/image_raw -n 1
    ```   
    
---


## Detecting a Line

In this next example, we're going to place our robot on the ground.  There is a yellow line painted on the ground, but no obstacles.


### Terminal 1: Open Gazebo and place a Husky on the test course:

    ```	
    cd ~/catkin_ws/src/followbot/
    roslaunch followbot course.launch
    ```
    
    - This is a customized `.launch` file, located in `~/catkin_ws/src/followbot/worlds`.
    - If the yellow track doesn't appear, try the following:
        ```
        export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/followbot/worlds
        ```
        	
### Terminal 2:  Launch a ROS node where we filter the camera image:
    ```
    cd ~/catkin_ws/src/followbot/scripts
    rosrun followbot camera_color_filter.py 
    ```	
   
    - This Python script has been modified slightly to show the original and filtered images.
    - If you get an error indicating that `rosrun` cannot find an executable:
        ```
        chmod +x camera_color_filter.py
        ``` 
--- 

## Following a Line

### Terminal 1: Open Gazebo and place a Husky on the test course:

    ```	
    cd ~/catkin_ws/src/followbot/
    roslaunch followbot course.launch
    ```
    
### Terminal 2:  Launch a ROS node that will send Twist commands to follow the line
    ```
    cd ~/catkin_ws/src/followbot/scripts
    rosrun followbot follower_p.py 
    ```	
    - If you get an error indicating that `rosrun` cannot find an executable:
        ```
        chmod +x follower_p.py
        ``` 
