🐶 Innovox - The Useless Guide Dog Project 🐾


Team Innovox

    Team Lead: Ashish Shajan - Saintgits
    Member 2: Muhammed Ajmal - Saintgits
    Member 3: Tomin Michael - Saintgits

Project Description
Guide Dog in the Supermarket:

Ever dreamed of having a guide dog navigate the aisles of a supermarket for you? Probably not, but here we are! Our project, in all its quirky glory, introduces a virtual guide dog in a simulated supermarket, ready to assist you in locating products—even the ones you weren’t looking for.
The Problem (That Doesn’t Really Exist)

Imagine this: you’re lost in a supermarket, searching for that elusive jar of peanut butter you forgot you needed. Sure, you could ask a human—but wouldn’t it be cooler if a virtual guide dog was there to assist?
The Solution (Nobody Really Asked For)

Introducing… Supermarket Guide Dog 3000! Leveraging ROS 2 and Gazebo simulations, our virtual canine will navigate the supermarket and "guide" you to products with a blend of enthusiasm and randomness. Yes, it might just lead you to the shampoo section instead of the cereal aisle.
Technical Details
Technologies/Components Used

For Software:

    Languages: Python, C++
    Frameworks: ROS 2 (Robot Operating System)
    Libraries: OpenCV (for item detection), Machine Learning for object recognition
    Tools: RViz (for visualization), Gazebo (for simulation), Python scripts for navigation

For Hardware (Simulated):

    Simulated Components: Robotic sensors, motors for virtual “dog” movement
    Specifications: Designed for navigating supermarket aisles with charm
    Requirements: A computer with ROS 2, Gazebo installed, patience, and a sense of humor.

Implementation

For Software:

    Simulation Setup: We created a supermarket environment in Gazebo for the guide dog’s adventures.
    Guide Dog Navigation: The virtual dog roams the aisles using ROS 2 nodes for pathfinding and avoiding “customers” (obstacles).
    Detection and Guidance: Machine learning algorithms recognize certain items and steer the guide dog toward them—even if they weren’t requested.
    Visualization in RViz: Watch our guide dog’s journey unfold in RViz for a play-by-play experience.

For Hardware (Simulated):

    Sensors & Actuators: Virtual sensors enable movement, obstacle avoidance, and item detection.
    Dog Model: An adorable dog avatar, because every guide dog must be cute.

Installation

    Clone the Project:

    bash

git clone https://github.com/TominMichael/guidedog.git
cd guidedog

Install Dependencies:

bash

sudo apt-get update
sudo apt-get install ros-foxy-desktop  # or appropriate ROS 2 version

Run Setup Scripts (inside the repository):

bash

    source /opt/ros/foxy/setup.bash  # or your installed ROS version
    colcon build
    source install/setup.bash

Running the Simulation

    Launch Gazebo World:

    bash

ros2 launch guidedog supermarket_world.launch.py

Start Guide Dog Node:

bash

ros2 run guidedog guide_dog_node

Launch RViz for Visualization:

bash

    ros2 launch guidedog rviz_view.launch.py

Project Documentation

For additional details, including node structure, software architecture, and use cases, see the README.md file and documentation within the repository.

Team Contribution

  Ashish Shajan - creation of dog urdf, launching in rviz and Gazebo
  Muhammed Ajmal - World  creation in Gazebo
  Tomin Michael - Integration of dog model in world




