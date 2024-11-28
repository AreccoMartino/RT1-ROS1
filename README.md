# Turtlesim Control with UI and Distance Monitoring

 ![Descrizione dell'immagine](https://blogger.googleusercontent.com/img/b/R29vZ2xl/AVvXsEiL8Il-zkRrexK9ce_4fZm-Y7MSTMPnQc65Ts35oHA6pESgXDYmiMSkBV5k4asnuekT0OB5SJmH4exPkA2Hjl91LUl19BCrYEa3A6pJgI5eBklNW4yi8kQsyHdjjLlJe3lBmTUUKFaBCKM/s400/images.png)    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       ![Descrizione dell'immagine](https://pokestop.io/img/pokemon/squirtle-256x256.png) 



## How to use turtlesim nodes

### 0. Clone the repository

Please move to your workspace and then in the src folder clone the repository

```bash
git clone <URL_of_this_repository>
```

Following the instructions the final state of the path will be /.../<your_workspace>/src/assignment1_rt/ and in assignment1_rt will be placed all the necessary files.

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
rosrun assignment1_rt UI.py
```
Replace <package_name> with the actual name of your ROS package.

Run the Distance node:

In the second terminal, go to the workspace src directory and start the Distance node with the following command:

```bash
rosrun assignment1_rt Distance.py
```
Again, replace <package_name> with the name of the package in which Distance.py is located.

## Node operation
### UI node (UI.py):
The UI.py node provides a simple text-based user interface for controlling the movement of turtles in the Turtlesim environment. The user can interactively choose which turtle to control, set velocity values, and send movement commands. Here's how it works:

*----- Spawning a Second Turtle: -----*

The node uses the /spawn service to create a new turtle named turtle2 in the Turtlesim environment at a specified position.

*----- User Selection of Turtle: -----*

The user is prompted to choose which turtle to control: turtle1 (default turtle) or turtle2 (spawned turtle).
Based on the user's input, the node sets up a ROS Publisher to send velocity commands to the chosen turtle (/turtle1/cmd_vel or /turtle2/cmd_vel).

*----- Velocity Configuration: -----*

The user is asked to input values for:
Linear velocity in the x-direction (Vx).
Linear velocity in the y-direction (Vy).
Angular velocity around the z-axis (Wz).
The values are validated to ensure they are numeric.

*----- Publishing Velocities: -----*

The node publishes the configured velocities to the chosen turtle for 1 second using the Twist message type.
After 1 second, the turtle is stopped automatically by publishing a Twist message with all velocity values set to zero.

*----- Iterative Control: -----*

After stopping the turtle, the program allows the user to select another turtle or reconfigure velocity values for continued control.

### Distance node (Distance.py):
The Distance.py node monitors and manages the behavior of two turtles (turtle1 and turtle2) in the Turtlesim simulation. It ensures that the turtles maintain safe distances from each other and stay within the simulation boundaries.
The main features are summarised below:

*----- Distance Monitoring and Control: -----*

Computes the distance between turtle1 and turtle2 using their positions.
If the distance is less than a threshold (e.g., 1.0), both turtles are stopped. Temporary scaled velocities are prepared to move them apart once the distance becomes safe again.

*----- Boundary Checking and Correction: -----*

Ensures that both turtles stay within the valid area of the Turtlesim world (x and y coordinates between 1 and 10).
If a turtle crosses the boundary, it is teleported back to the nearest valid position using the /teleport_absolute service.

*----- Velocity and Movement Management: -----*

Publishes Twist messages with inverted velocities to move turtles apart when resolving proximity issues.
Uses the teleport service to handle boundary violations, resetting turtles to safe positions.

*----- Timers for Periodic Checks: -----*

Implements periodic distance and boundary checks using ROS timers, ensuring continuous monitoring in real-time.
