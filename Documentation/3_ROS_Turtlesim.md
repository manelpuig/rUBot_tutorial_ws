# **ROS Tutorial Turtlesim Script Programming**
Let's now create our proper Turtlesim package with different python nodes to perform simple navigation control exemples of Turtlesim robot.

This is performed following the tutorial:
- http://wiki.ros.org/turtlesim/Tutorials
- http://wiki.ros.org/turtlesim/Tutorials#Practicing_Python_with_Turtlesim
- http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
- https://github.com/Apress/Robot-Operating-System-Abs-Begs

## **Control of Turtlesim**
A specific package "turtlesim_tutorial" is created where different motion control programs will be done.

The package is already created, but we remind you how to create a "turtlesim_tutorial" package with dependencies (rospy, geometry_msgs, turtlesim)

In src directory:
```shell
cd src
catkin_create_pkg turtlesim_tutorial rospy geometry_msgs turtlesim
cd ..
catkin_make
```
We have also created folders:
- "scripts" where we have placed python programs for speciffic motion control functions. 
- "launch" where execution programs are placed

Let's perform different exercises to understand how to programm the motion control functions in ROS environment.

### **Exercise: Control turtlesim**

In this exercise, a node "/move_turtle" can be created to communicate with the "/turtlesim" node.

This node "/move_turtle" created performs a speciffic motion control function:
- publishes in the /turtle1/cmd_vel topic a message Twist

The node "/turtlesim" performs:
- Subscribes to the "/turtle1/cmd_vel" topic and reads the Twist message
- Executes the movement
- Publishes to the "/turtle1/Pose" topic the Position and orientation of the turtlesim robot on the output terminal

![](./Images/3_Turtlesim/01_move_turtle0.png)

To properly run the ROS application, create a launch folder containing the "turtlesim_control.launch" file:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <node pkg="turtlesim_tutorial" type="move_turtle.py" name="move_turtle"/>
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>
</launch>
```
To launch the exercise, type:
```shell
cd rUBot_tutorial_ws
roslaunch turtlesim_tutorial turtlesim_control.launch
rqt_graph
```
### **Exercise: Move distance with turtlesim**
Develop a python script to perform the following functionalities.
- Control the Turtlesim movement inside a room with a wall in a distance X direction and another wall in a distance in Y direction.
- Specify a maximum distance in x and y direction for turtlesim to avoid hitting the walls.
- node "/move_turtle":
    - subscribes to "/turtle1/Pose" topic to read the POSE message
    - if the Pose.x or Pose.y is lower than the distance specified, publish linear and angular speed
    - if the Pose.x or Pose.y is higher, then stop

![](./Images/3_Turtlesim/01_move_turtle.png)

Create in "scripts" folder the python file "move_distance.py"

Create in "launch" folder a launch file "move_distance.launch" to run all nodes with speciffic useful parameters:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
        <arg name="v" default="0.5"/>
        <arg name="w" default="0"/>
        <arg name="d" default="7"/>
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node"/>
    <node pkg="turtlesim_tutorial" type="move_distance.py" name="move_turtle" output="screen" >
        <param name="v" value="$(arg v)"/>
        <param name="w" value="$(arg w)"/>
        <param name="d" value="$(arg d)"/>
    </node>
</launch>
```

To execute it, type:
```shell
roslaunch turtlesim_tutorial move_distance.launch
```
![](./Images/3_Turtlesim/02_turtlesim_move_dist1.png)
![](./Images/3_Turtlesim/03_turtlesim_move_dist2.png)

### **Proposed Exercise: Move turtlesim within a predefined square room**
Create a new "move_inside_room.py" node, modifying the "move_distance.py" node to:
- Define a square room limits in X and Y
- Move with a generic twist vector
- Subscribes to "/turtle1/Pose" topic to read the POSE message
    - if the Pose.x and Pose.y is inside the X and Y limits, publish linear and angular speed
    - if the Pose.x or Pose.y is outside the X and Y limits, then stop

### **Proposed Exercise: Move move during time interval with turtlesim**
Create a new "move_time.py" node, modifying the "move_distance.py" node to:
- Move with a generic twist vector
- during a time interval

Useful indications:
- use the structure:
````python
def move_rubot(lin_velx,ang_vel,time_duration):
    ...
    time_begin = rospy.Time.now()
    while ...:
        if (duration_s <= time_duration):
            rospy.loginfo("Robot running")
            ...
        else:
            rospy.logwarn("Stopping robot")
            ...
        time_end = rospy.Time.now()
        rospy.loginfo("Time_end = " + str(time_end))
        duration = time_end - time_begin
        duration_s = duration.to_sec()
````
### **Exercise: Go to target point with turtlesim**
Develop a ROS node to perform the following functionalities.
- Specify a target point (x,y) with a tolerance for turtlesim to move
- Read the Pose of turtlesim
- if the turtlesim Position is far from the point, publish linear and angular speed
- if it is close within the tolerance, stops

![](./Images/3_Turtlesim/01_move_turtle.png)

We can describe graphically how we will control the turtlesim:

![](./Images/3_Turtlesim/04_turtlesim_w.png)

We can use **Object Oriented Programming**, creating a "TurtleBot" python Class and use input parameters to specify the target point and tolerance

Object Oriented Programming (OOP) is everywhere in python. 
- OOP is a way to organize your code into Classes. 
- A Class a sort of container of objects with a set of attributes (variables/properties) and methods (functions)

Create a "/move_turtle" node with go2point functionality. This is defined in the "go2point.py" python file.

Create a "go2point.launch" launch file to setup all needed nodes:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
        <arg name="x" default="1"/>
        <arg name="y" default="1"/>
        <arg name="tol" default="0.1"/>
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node"/>
    <node pkg="turtlesim_tutorial" type="go2point.py" name="move_turtle" output="screen" >
        <param name="x" value="$(arg x)"/>
        <param name="y" value="$(arg y)"/>
        <param name="tol" value="$(arg tol)"/>
    </node>
</launch>
```
To execute, type:
```shell
roslaunch turtlesim_tutorial go2point.launch
```
![](./Images/3_Turtlesim/06_turtlesim_go2point1.png)


### **Activity 2: ROS turtlesim go2pose**

The objective is to reach the desired position with the proper orientation also.

Create a new "go2pose.py" node, modifying the "go2point.py" node to:
- reach first the position 
- and later the orientation
