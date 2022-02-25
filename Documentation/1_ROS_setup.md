# **rUBot_Tutorial_ws**
This is a first tutorial to learn ROS basics.

You will need to :
- setup ROS Noetic in your PC
- Prepare your Workspace 

## **1. Setup ROS Noetic in you PC**
You have diferent options:
### **Use "Aula Virtual"**
All the SW is installed and ready to work

Follow the link: http://www.ub.edu/aules-virtuals/treballo-amb-windows/

### **Install ROS Noetic in your PC**
You have first to install VirtualBox: https://www.virtualbox.org/wiki/Downloads

Then you need to install a Virtual Machine with Ubuntu 20 and ROS Noetic. You have two options:
- Download a preinstalled ROS Noetic Virtual Machine from here: https://ubarcelona-my.sharepoint.com/:u:/g/personal/manel_puig_ub_edu/EYDtbVLrS7tBrU54EgVwfF8BWPZ6IYuBEUQGwZFTs6Y4PQ?e=BdfKhX
- Custom installation:
    - Dowload Ubuntu 20.04: (https://ubuntu.com/download/desktop)
    - Install the Ubuntu20 VM (Video Link: https://www.youtube.com/watch?v=QbmRXJJKsvs
https://linuxhint.com/install_ubuntu_18-04_virtualbox/)
    - Start the Ubuntu20 virtual machine and update it
    - Install ROS Noetic following the instructions: http://wiki.ros.org/noetic/Installation/Ubuntu

## **2. Prepare your Workspace**
Once you are in the ROS Noetic Virtual machine, you can:
- create your own workspace
- Use an existing workspace

### **2.1 Create your own workspace**
This is the best option to learn ROS. You will create a new repository and you will follow the instructions to create all the packages to fulfill the objectives and functionalities.

- In your github account, create a new public repository
- add gitignore with ROS option
- Sync this repository locally in your Ubuntu ROS Noetic session following the instructions in the next section.

### **2.2 Use an existing workspace**
You can also use our repository and make your modifications according to the exercises we will propose you. 
In that case, you can follow the instructions:

- Fork the "rubot_tutorial_ws" repository from my github
![](./Documentation/Images/1_fork.png)

- Create a new sync permissions (Personal Access Tokens)
![](./Documentation/Images/1_tokens.png)
- Open your virtual machine Ubuntu20 with ROS Noetic and clone your forked directory in your Desktop (copy the exact url to your forked repository)
```shell
git clone https://github.com/yourusername/rUBot_tutorial_ws
```
- you will need to insert the username and the saved PAT password
- Compile:
```shell
cd ~/Desktop/rubot_tutorial_ws
catkin_make
```
- Open .bashrc file
```shell
gedit ~/.bashrc
```
- Ensure that you have the last 2 lines (review the exact name of your repository):
```xml
source /opt/ros/noetic/setup.bash
source ~/Desktop/rUBot_tutorial_ws/devel/setup.bash
```
- You are ready to work with your repository for this session

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