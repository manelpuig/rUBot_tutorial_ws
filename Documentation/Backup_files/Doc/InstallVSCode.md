# **Install VS Code**
You can install VS Code with different methods

## **a) Using Installation file**

- Go to the VSCode web page
- download the execution file for the OS version of your system
- Install

## **b) Using APT**
Here's how you can install VS Code on Ubuntu 20.04:

1. **Download the Repository Key**:

Open a terminal (Ctrl+Alt+T) and run the following command to import the Microsoft GPG key used for signing packages:

```bash
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo mv packages.microsoft.gpg /usr/share/keyrings/
```

2. **Add the Repository**:

Run the following command to add the VS Code repository to your system:

```bash
echo "deb [signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
```

3. **Update Package List**:

Update the package list to include the newly added repository:

```bash
sudo apt update
```

4. **Install VS Code**:

Finally, install Visual Studio Code with the following command:

```bash
sudo apt install code
```

5. **Start VS Code**:

You can start VS Code by searching for it in the application menu or by running the `code` command in the terminal.

That's it! You should now have Visual Studio Code installed on your Ubuntu 20.04 system.

## **c) Using SNAP**

You can also install Visual Studio Code using Snap, a package management system for Linux distributions. Here's how to install VS Code using Snap on Ubuntu 20.04:

1. **Open Terminal**:

Press Ctrl+Alt+T to open a terminal.

2. **Install Snap (If Not Installed)**:

If you haven't installed Snap, you can do so using the following command:

```bash
sudo apt update
sudo apt install snapd
```

3. **Install Visual Studio Code**:

To install Visual Studio Code using Snap, use the following command:

```bash
sudo snap install --classic code
```

The `--classic` flag is used because VS Code requires classic confinement to access certain system resources.

4. **Start Visual Studio Code**:

After the installation is complete, you can launch Visual Studio Code by simply typing `code` in the terminal, or you can search for "Visual Studio Code" in the application menu.

That's it! You've successfully installed Visual Studio Code using Snap on Ubuntu 20.04.