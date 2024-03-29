# Followbot
This project comes from Chapter 12 of the textbook, but modified significantly to work with our Husky.

The goal is to direct a Husky to drive around a course/track, characterized by a line of a certain color.

---
## The `followbot` ROS Package
First, we need to create a ROS "package" for this project.  

We will assume that you have cloned the course GitHub respository to `~/Projects/IE-482-582/spring2024`, and that your ROS packages will be installed in `~/catkin_ws/src`.

**UPDATE 2024-03-27:**
- We are going to use a different approach to sync this code between our GitHub clone of the `followbot` package (located in `~/Projects/IE-482-582/spring2024/chapter_12_followbot`) with our `catkin_ws` package (located in `~/catkin_ws/src/followbot`).

1.  Make a backup copy of your existing `catkin_ws/src/followbot` package (if it exists):
    ```
    cp -r ~/catkin_ws/src/followbot/ ~/Desktop/followbotbackup
    ```

2.  Delete the old followbot package (if it exists):
    ```
    rm -r ~/catkin_ws/src/followbot/
    ```
    
3.  Create a "symbolic link" to the files in the GitHub repo:
    ```
    ln -s ~/Projects/IE-482-582/spring2024/chapter_12_followbot ~/catkin_ws/src/followbot
    ```
    - The pattern for this is `ln -s [path to actual files] [path to where link will be placed]`
    - *NOTE*: If you try to run the `ln -s ...` command and `~/catkin_ws/src/followbot` already exists, you'll get a `chapter_12_followbot` folder within `~/catkin_ws/src/followbot`.
        - That's not what you want!
        
4.  Double check that the symbolic link worked:
    ```
    ls ~/catkin_ws/src/followbot
    ```
    It should return
    > ```
    > CMakeLists.txt  package.xml  README.md  scripts  worlds
    > ```
        
    If you get something different, go back to Step 2.
                
5.  Build the catkin workspace
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


---

# Modifying the Course - Two Lines

This is a (modified) summary of the steps we followed in class to create a two-line course.
- We used `GIMP` to edit the `course.png` file...those steps are not repeated here.

**NOTE: This repo already contains the modified files (`course_duo.launch`, `course_duo.world`, `course_duo.material`, and `course_duo.png`).  The steps below describe how to create those files.**

1. First, change directories to where our worlds are saved:
    ```
    cd ~/catkin_ws/src/followbot/worlds
    ```

2.  Create `course_duo.launch` as a copy of `course.launch` and edit:
    ```
    cp course.launch course_duo.launch
    pico course_duo.launch
    ```
    
    Replace the value of the `world_name` argument to reference `course_duo.world`, as follows:
    ```
        <arg name="world_name" value="$(find followbot)/worlds/course_duo.world"/>
    ```
    
    
3.  Create `course_duo.world` as a copy of `course.world` and edit:
    ```
    cp course.world course_duo.world
    pico course_duo.world
    ```
    
    Scroll down near the bottom and edit the lines inside the `<material>`...`</material>` tags.  We need to reference `course_duo` instead of `course`:
    ```
          <material>
            <script>
              <uri>file://worlds/course_duo.material</uri>
              <name>course_duo</name>
            </script>
          </material>    
    ```
    - NOTE:  In class, we forgot to change the `<name>course_duo</name>` line.

4. Create `course_duo.material` as a copy of `course.material` and edit:
    ```
    cp course.material course_duo.material
    pico course_duo.material
    ```
    
    The file should look like this:
    ```
    material course_duo
    {
      receive_shadows on
      technique
      {
        pass
        {
          ambient 0.5 0.5 0.5 1.0
          texture_unit
          {
            texture course_duo.png
          }
        }
      }
    }
    ```
      
    - NOTE:  In class, we forgot to edit the first line of this file. 
    
Now, you're ready to launch:
```
roslaunch followbot course_duo.launch
```    
