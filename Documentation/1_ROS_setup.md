# **rUBot_Tutorial_ws**

First of all we will learn how to install the ROS Noetic in your computer from a Docker utility.

## **1. ROS Installation & Tools**
You have to install:

a) Docker for windows: an easy to install application that enables you to manage your containers (https://docs.docker.com/desktop/install/windows-install/)

    >   Open PowerShell terminal and type: systeminfo | find "Tipo de sistema"
    >
    >   The output has to be: x64-based PC

- Then you have to complete your installation with WSL 2 for kernell update in: https://learn.microsoft.com/ca-es/windows/wsl/install-manual#step-4---download-the-linux-kernel-update-package
- Stablish WSL2 as default by opening a powershell and typing: wsl --set-default-version 2
- restart your computer

b) Xming X server for Windows, an X11 display server that will allow you to visualize, in particular the RViZ robot visualization and tools and the Gazebo simulation of the robot. (https://sourceforge.net/projects/xming/)

c) Visual Studio Code

### **1.1 ROS1 Images**

Official images are:
- ROS Noetic: osrf/ros:noetic-desktop-full
- Open PowerShell and type: 
```shell
docker pull osrf/ros:noetic-desktop-full
```
To create a Container with GUI and folder share, open a terminal and type:
```shell
docker run --name ROS1_Noetic_mpuig -e DISPLAY=host.docker.internal:0.0 --mount src="C:\Users\puigm\Desktop\ROS\shared_folder",dst=/home/myDocker_shared,type=bind -it osrf/ros:noetic-desktop-full
```

### **1.2. XLaunch**
For graphical interface, open Xlaunch:
- First choose “Multiple windows” and set to 0 the “Display number”
- Then, set “Start no client” in the second screen
- In the third screen, click “Clipboard” and Primary Selection and “Disable access control”, while leaving “Native opengl” unclicked
- And just click “Finalize” in the last screen

### **1.3. VS Code**

We will use VS Code to sync a copy of your github repository in your local PC:
- Open VS Code and choose "Clone repository"
- type the github link of the desired repository
- select the local destination folder

We will also use VS Code to work and sync the changes we have made in Docker Container:
- Install in your Visual Studio Code the extensions:
    - Docker
    - Dev Containers
- Open Docker Desktop
- Execute the container and Connect to it within VS Code:
    - from left-side menu choose Docker
    - right-click on the running container and select "Attach VS Code"
- Update your docker Container:
    ```shell
    apt update
    apt upgrade
    ```
    > repeat these instructions until you see "All packages are up to date"

- Install some functionalities:
    ```shell
    apt install -y git && apt install -y python3-pip
    ```

## **2. Prepare your Workspace**
Once you are in the ROS Noetic Virtual machine, you can:
- create your own workspace
- Use an existing workspace

### **2.1 Create your own workspace**
This is the best option to learn ROS. You will create a new repository and you will follow the instructions to create all the packages to fulfill the objectives and functionalities.

- In your github account, create a new public repository
- add gitignore with ROS option


### **2.2 Use an existing workspace**
You can also use our repository and make your modifications according to the exercises we will propose you. 
In that case, you can follow the instructions:

- Fork the "rubot_tutorial_ws" repository from my github
![](./Images/1_fork.png)

- Open your Docker container with ROS Noetic 
- Using VS Code, connect to the ROS1_Noetic running container
- Clone your forked directory in your home directory of container
    ```shell
    git clone https://github.com/yourusername/rUBot_tutorial_ws
    ```
- Open .bashrc file (from root) with VS Code (open file...)
- Ensure that you have the last 2 lines (review the exact name of your repository):
    ```xml
    source /opt/ros/noetic/setup.bash
    source home/rUBot_tutorial_ws/devel/setup.bash
    cd /home/rUBot_tutorial_ws
    ```
- open a new terminal and Compile:
    ```shell
    cd /home/rUBot_tutorial_ws
    catkin_make
    ```
- You are ready to work with your repository for this session

## **1.3. Repository syncronisation**
When working on VS Code, you need to:

- Select "source control" from left side menu
- select changes to sync
- Add a commit
- Push

Alternativelly you could:
- Use github web editor
- use github Desktop