# Wanderbot -- Chapter 7


## Installation Instructions

NOTE: These instructions are a little different from the textbook's.

1.  Create the Package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg wanderbot rospy geometry_msgs sensor_msgs
    ```
    *Note: We have added some dependency packages.  We'll take a look at `CMakeLists.txt` and `package.xml` shortly to see how that helped us.*
    
2. Let's go ahead and create our `scripts` directory:
    ```
    cd ~/catkin_ws/src/wanderbot
    mkdir scripts
    ```
        
3. Make sure you have the latest code from the course GitHub site.  
    - See [GitHub Cheatsheet](../../notes/git_cheatsheet.md) for instructions.
        
 4. Copy the Python scripts to our wanderbot workspace
    ```
    cd ~/Projects/IE-482-582/spring2024/Textbook/chapter_07_wanderbot
    cp scripts/* ~/catkin_ws/src/wanderbot/scripts/
    ```
    
 5. Make our Python scripts executable
    ```
    cd ~/catkin_ws/src/wanderbot/scripts
    chmod +x *.py
    ```
    
6. Compile/make our package

    ```
    cd ~/catkin_ws
    catkin_make
    ```
        
---

## Red Light / Green Light
In our first script, we'll let the robot drive for 3 seconds, then stop for 3 seconds, then repeat.

### Running the script:
We'll need two (2) terminal windows.

1. Use a *launch* file to start roscore and Gazebo.

    ```
    export TURTLEBOT3_MODEL=burger 
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

2. Run the red light / green light script:

    ```
    cd ~/catkin_ws/src/wanderbot/scripts
    rosrun wanderbot red_light_green_light.py
    ```
    
    - The textbook uses `./red_light_green_light.py cmd_vel:=cmd_vel_mux/input/teleop` to run the executable Python script.  Open the file and notice that it starts with `#!/usr/bin/env python3`.  
    - I suggest you explicitly use `rosrun [packagename] [scriptname]` instead.
    - The `cmd_vel:=cmd_vel_mux/input/teleop` parameter is a mapping, which essentially says "everywhere you see `cmd_vel`, replace this with `cmd_vel_mux/input/teleop`.  The reason for the replacement is that the original turtlebot used in our textbook used that topic name.  Our turtlebot3 uses just `cmd_vel`.

### Understanding what's happening
1. See the textbook (P. 100) for an explanation of the code.  We're now going to look at some other elements of the package.
2. Take a look at `CMakeLists.txt`.  Notice that `geometry_msgs`, `rospy`, and `sensor_msgs` are included.
3. In Line 3 of `red_light_green_light.py`, we have `from geometery_msgs import Twist`.  What is Twist?
    - `roscd geometry_msgs`
    - Find the `Twist` message definition
4. Insert the following print statement somewhere before the while loop:
    ```
    print(green_light_twist)
    ```
5. Edit the code to make the robot rotate 45-degrees clockwise.  Make it rotate 45-degrees counter clockwise.  Can you make the robot flip over?

6. Let's find the launch file and see what it's doing.
    ```
    roscd turtlebot3_gazebo
    ls
    cd launch
    pico turtlebot_world.launch
    ```
    
7. Let's see what information is being broadcast.  Open another terminal window and try:
    ```
    rostopic list
    ```
8. We can also create a graph of the network:
    ```
    rqt_graph
    ```
    
---

## Reading Sensor Data
In the next script, we're going to capture data from our robot's laser scanner.

### Running the script:
We'll need two (2) terminal windows.
    
1. Use a *launch* file to start roscore and Gazebo.
    
    ```
    export TURTLEBOT3_MODEL=burger 
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```

    
2. Run the range ahead script:
    
    ```
    cd ~/catkin_ws/src/wanderbot/scripts
    rosrun wanderbot range_ahead.py
    ```

### Understanding what's happening

1. Open another terminal window:
    ```
    rostopic list
    rostopic echo scan
    ```
    
2. Move the robot around in Gazebo.  Watch what is happening with the scan topic (in the other terminal window).

3. Let's add some print statements to our range ahead script to see what info we have available.

4. We'll launch rviz in another terminal:
    ```
    rosrun rviz rviz
    ```
   Take a look at Chapter 8 to see how to configure rviz.
   
---

## Wanderbot

Finally, let's run the script that moves the robot around aimlessly, without bumping into things:

### Running the script:
We'll need two (2) terminal windows.

1.  Use a *launch* file to start roscore and Gazebo.

    ```
    export TURTLEBOT3_MODEL=burger 
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
    ```
    
    - NOTE:  The textbook references `roslaunch turtlebot_gazebo turtlebot_world.launch`.  The `turtlebot_gazebo` package has been replaced by `turtlebot3_gazebo`.

2. Run the wander script:

    ```
    cd ~/catkin_ws/src/wanderbot/scripts
    rosrun wanderbot wander.py
    ```
    



