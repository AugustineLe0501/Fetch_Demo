# Fetch_Demo
Move Fetch robot arm to pick the dice and place into the tray

# Download dependencies 

Fetch ros: https://github.com/fetchrobotics/fetch_ros

Moveit: https://moveit.ros.org/install/source/

Moveit Python: https://github.com/mikeferguson/moveit_python

# Run the Demo
Step 1: Connect with Fetch Wifi 

        ID: Fetch-Wifi
        Password: robotics
        
Step 2: Export Ros master URI and change Host Name (This task need to do every time you open the new terminal)

        ~$ ping fetch31.local (Check the connection)
        ~$ export ROS_MASTER_URI=http://fetch31.local:11311 
        ~$ export ROS_HOSTNAME="Your_host_name" (If you do not know your host name, command "~$ hostname -I" in the terminal)
        
Step 3: Roslaunch MoveIt, Ar_tracking, and 
        
        ~$ roslaunch fetch_moveit_config move_group.launch
        ~$ roslaunch ar_tracking_alvar fetch_ar_tag.launch
        ~$ rosrun controller main.py



