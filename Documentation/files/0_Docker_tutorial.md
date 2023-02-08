# **Docker tutorial**

ROS Installation will be made using Docker

## **1. Docker Installation & Tools**
We will learn:
- Installation
- Starting docker ROS official images
- Docker getting starter
- Docker with GUI
- Creating custom Docker
- Githuib sync from docker

### **1.1 Docker Installation**
Installation instructions could be found in:
- http://wiki.ros.org/docker
- http://wiki.ros.org/docker/Tutorials/Docker
- https://docs.docker.com/get-docker/

In windows:
- download and install: https://docs.docker.com/desktop/install/windows-install/

    >   Open PowerShell terminal and type: systeminfo | find "Tipo de sistema"
    >
    >   The output has to be: x64-based PC
- restart your computer
- Open Docker Desktop
- Then you have to complete your installation with WSL 2 for kernell update in: https://learn.microsoft.com/ca-es/windows/wsl/install-manual#step-4---download-the-linux-kernel-update-package
- Stablish WSL2 as default by opening a powershell and typing: wsl --set-default-version 2
You need also to install Xlaunch for windows for GUI:
- https://sourceforge.net/projects/vcxsrv/

### **1.2 Starting docker ROS official images**
You can find a lot of ROS images available. 

Usefull **images that could be opened with browser**:
- ROS_Noetic: https://hub.docker.com/r/arvinskushwaha/ros-noetic-desktop-vnc/tags
- ROS2_Foxy: https://hub.docker.com/r/husarion/ros2-desktop-vnc

    **Some important trics:**
    - To copy and paste text use the clipboard
    - For home symbol use the "Extra keys"
    - To copy files or folders from/to windows, open a Power Shell terminal and type:
    ```shell
    docker cp c:/Users/puigm/Desktop/road1 Ros1_Noetic:/home/ubuntu/rUBot_mecanum_ws/src/rubot_mecanum_description/models
    docker cp Ros1_Noetic:/home/ubuntu/rUBot_mecanum_ws/src/rubot_mecanum_description/worlds/road1.world c:/Users/puigm/Desktop
    ```
The **official images** are mantained by Open Source Robotics Foundation (OSRF).

You can find them in:
- https://registry.hub.docker.com/_/ros/
- https://hub.docker.com/r/osrf/ros/tags

Interesting information is in:
- https://github.com/noshluk2/ros1_wiki/tree/main/docker

Let's install ROS1 and 2 official image:
```shell
docker pull osrf/ros:noetic-desktop-full
docker pull osrf/ros:foxy-desktop
```
To see all the images installed:
```shell
docker images -a
```
Creating a interactive container from image
```shell
docker run -it osrf/ros:noetic-desktop-full
```
Giving Name to a container while creating
```shell
docker run --name ROS1_Noetic_osrf -it osrf/ros:noetic-desktop-full
```
List of containers:
```shell
docker ps
```
Connect shell to running container
```shell
docker exec -it (container_id) bash
```
Close the container deleting it from Docker Desktop

### **1.3 Docker getting starter**
Lets see a simple Talker-Listener exemple

Run a container with a speciffic name
```shell
docker run --name ROS1_Noetic_osrf -it osrf/ros:noetic-desktop-full
roscore
```
Open a second terminal. Start listener:
```shell
docker exec -it (container_id) bash
source /opt/ros/noetic/setup.bash
rosrun rospy_tutorials listener
```
Open a thirth terminal. Start talker:
```shell
docker exec -it (container_id) bash
source /opt/ros/noetic/setup.bash
rosrun rospy_tutorials talker
```
Stop  container
```shell
docker kill (container_id)
```
### **1.4 Docker with GUI**

Running a container with GUI enabled for Windows. We need to create an environment to display the emerging grafic windows.

We need to open XLaunch program and specify the DIPLAY 0. Then run a Docker with this environment:
```shell
docker run --name ROS2_Foxy_osrf -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:foxy-desktop
```
Run the Master
```shell
roscore
```
Start another terminal in the same container ID and open turtlesim node.
```shell
docker exec -it (container_id) bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```
> Xlaunch has to be opened in lower task menu. Ik errors you have to delete the file CVXServer.0 in C:\Users\HP\AppData\Local\Temp. For that you have to close all applications and restert the computer.

### **1.5. Creating custom Docker**

In Visual Studio code, install the docker extensions:
- Docker (Microsoft)

Then create a new "Dockerfile" with the image configuration:
```python
# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:foxy-desktop

RUN sudo apt-get clean
RUN apt-get update
RUN sudo apt-get upgrade -y
RUN apt-get install -y git && apt-get install -y python3-pip
RUN echo "git and pip Installed"
RUN apt install -y gedit
RUN apt install -y gedit-plugins
RUN echo "gedit Installed"
RUN apt install nautilus -y
RUN apt install gnome-terminal -y
RUN sudo apt install nautilus-actions gnome-terminal -y
RUN echo "Nautilus File manager Installed"

RUN cd /home/

RUN echo "ALL Done"
```
Open a Power Shell terminal in the Dockerfile location and type:
```shell
docker build -t ros2_foxy_mpuig .
```
The Dockerfile is a text file that will produce a Docker Image

Run the image:
```shell
docker run --name ROS2_mpuig -e DISPLAY=host.docker.internal:0.0 -it ros2_foxy_mpuig:latest
```
Start another terminal in the same container ID and open turtlesim node.
```shell
docker exec -it (container_id) bash
```
Open terminator
```shell
nautilus
```
In terminal type:
```shell
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtlesim_node
```

### **1.6. Github sync from docker**

When finished, **syncronize** the changes with your github. 
- Open a terminal in your local repository and type the first time:
```shell
git config --global user.email mail@alumnes.ub.edu
git config --global user.name 'your github username'
git config --global credential.helper store
```
- for succesive times, you only need to do:
```shell
git add -A
git commit -a -m 'message'
git push
```
- you will need to insert the username and the saved PAT password
- syncronize your repository several times to save your work in your github account
> - You can **update** your repository either in your local or **remote repository**:
>   - Local: with the previous instructions
>   - Remote: using web-based Visual Studio Code:
>       - pressing "·" key
>       - performing repository modifications
>       - typing "**git pull**" to syncronize

