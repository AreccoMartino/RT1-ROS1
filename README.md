# Turtlesim Control with UI and Distance Monitoring

## How to use turtlesim nodes

### 1. Start roscore

To get started, it is necessary to start roscore, which is the heart of the ROS system. Open a terminal and type the following command:

```bash
roscore
```

### 2. Start turtlesim
Once roscore has started, open a second terminal. In this terminal, run this command to start the Turtlesim simulation node:

```bash
rosrun turtlesim turtlesim_node
```
This command must be launched in the workspace where the package was initialized.

### 3. Start the UI.py and Distance.py nodes
After starting turtlesim_node, you can start the nodes. To do this, open two separate terminals.

Run the UI node:

In the first terminal, go to the workspace src directory and start the UI node with the following command:

```bash
rosrun <package_name> UI.py
```
Replace <package_name> with the actual name of your ROS package.

Run the Distance node:

In the second terminal, go to the workspace src directory and start the Distance node with the following command:

```bash
rosrun <package_name> Distance.py
```
Again, replace <package_name> with the name of the package in which Distance.py is located.

## Node operation
### UI node (UI.py):
The UI node provides a simple interface for the user to control the movement of the turtles (turtle1 and turtle2).

### Distance node (Distance.py):
Observe the turtles and check their position to make sure they are in a safe position, otherwise, the node control them and establishes the safety.




Translated with DeepL.com (free version)
