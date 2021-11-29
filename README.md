# FinalProjectSFAS - Fall 2021
Group 1
Thorbjørn Pihl - s173851
Leila Sedighi - s200084
Giacomo Trovò - s202447
Anna Grillenberger - s213637

Commands to  start the simulation:

$ roslaunch final_project turtlebot3_world.launch

Parameters
1) enable_robot (default true): Will spawn the robot in the environment at launch
2) enable_competition (default true): Will spawn both the random barriers/obstacles and QR-markers
3) layout (default 0): Specifies the layout of the QR markers. You can use this to ensure the QR-markers are always spawned in the same location (e.g. when debugging). Can be either 0 (random), 1, 2, 3, or 4. Note that for the project you hand in, the layout will be 0 and thereby random!
4) gui (default true) & headless (default false): Gazebo is launched with a graphical user interface. You may choose to disable this GUI in favor of computational power and only visualize through other tools such as rviz.


Commands to read the QR code:

$ roslaunch final_project qr_visp.launch


Commands to import the map

$ roslaunch final_project navigation.launch

Commands to start the program

$ rosrun final_project explore_waypoints.py
