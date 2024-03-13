For Robot Preparation:

1. Download Raspberry Pi image Ubiquity from : https://ubiquity-pi-image.sfo2.digitaloceanspaces.com/2023-02-09-ubiquity-base-gdm3-focal-raspberry-pi.img.xz
2. Stop the RPi hostspot and connect to the internet for update.
3. Update the RPi image :```sudo apt update```
5. FYI, Ubiquity will continue update in background once RPi connected to the internet.
6. So, the best way to know the completion is completed by rebooting the RPi : ```sudo reboot```
7. Install samba : ```sudo apt install samba```
8. Run again Step #6 if it failed.
9. Go to /etc/samba and edit file name smb.conf with below addition:
   (a) ```wins support = yes```
   (b) ```[catkin_dk_src]
         comment = Samba on Ubuntu
         path = /home/ubuntu/catkin_dk/src
         read only = no
         browsable = yes```
10. Update Samba password : ```sudo smbpasswd -a ubuntu```
11. Enter new password whenever ask. (Prefer = ubuntu)
12. Restart robot : ```sudo reboot```
13. Install I2C library : ```sudo apt-get install libi2c-dev```
14. Setting up the path for roscore at this location : ```cd /etc/ubiquity/```
15. Edit file name ros_setup.bash : ```sudo nano ros_setup.bash```
16. Comment the 1st catkin_setup line and add NEW catkin_setup :```catkin_setup=/home/ubuntu/catkin_dk/devel/setup.bash && test -f $catkin_setup && . $catkin_setup```
17. Restart robot : ```sudo reboot```
18. Create catkin : ```mkdir -p ~/catkin_dk/src```
19. Go to catkin folder : ```cd catkin_dk```
20. Make the catkin : ```catkin_make```
21. Add 3 lines as below in .bashrc file
    (a) ```source ~/catkin_dk/devel/setup.bash```
    (b) ```export ROS_MASTER_URI=http://<robot_IP>:11311```
    (c) ```export ROS_HOSTNAME=<robot_IP>```
22. Load the new setup : ```source ~/.bashrc```
23. Go to src folder : ```cd ~/catkin_dk/src```
24. Git clone file : ```git clone https://github.com/mentor-dyun/ros-i2cpwmboard.git```
25. Back to catkin folder : ```cd ..```
26. Make the catkin : ```catkin_make```
27. Load the new setup : ```source ~/.bashrc```
28. Create ROS package in catkin_dk/src folder : ```catkin_create_pkg donkey_llc rospy```
29. Create Python script inside catkin_dk/src/donkey_llc/src and name as low_level_control.py <get the the copy in Tiziano github
30. Make the low_level_control.py executable : ```chmod +x low_level_control.py```
31. Back to catkin/src folder : ```cd ~/catkin_dk/src```
32. Git clone file : ```git clone https://github.com/ros-teleop/teleop_twist_keyboard.git```
33. Back to catkin_dk folder : ```cd ~/catkin_dk```
34. Make the catkin : ```catkin_make```
    
    -----FOR RASPBERRY PI 4 MODEL B------
1. Back to catkin_dk/src folder : ```cd ~/catkin_dk/src```
2. Git clone file : ```git clone https://github.com/UbiquityRobotics/raspicam_node.git```
3. Go to /etc/ros/rosdep/sources.list.d/ folder.
4. Create a file name as ```30-ubiquity.list```
5. Add this line into that file : ```yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml```
6. Update ROS repositoty : ```rosdep update```
7. Back to catkin_dk folder : ```cd ~/catkin_dk```
8. Run this code : ```rosdep install --from-paths src --ignore-src --rosdistro=noetic -y```
9. Make the file : ```catkin_make```
    
    -----FOR RASPBERRY PI 3 MODEL B+ ------
1. Back to catkin_dk/src folder : cd ~/catkin_dk/src
2. Git clone file : git clone https://github.com/dganbold/raspicam_node.git
3. Allocate 1GB swap area : sudo fallocate -l 1G /swapfile
4. Verify by : ls -lh /swapfile
   Output : -rw-r--r-- 1 root root 1.0G <timestamp> /swapfile
5. Enabling swapfile : sudo chmod 600 /swapfile
6. Verify by : ls -lh /swapfile
   Output : -rw------- 1 root root 1.0G <timestamp> /swapfile
7. Marking the swapfile : sudo mkswap /swapfile
8. Enabling swapfile : sudo swapon /swapfile
9. Making swapfile permanent : sudo cp /etc/fstab /etc/fstab.bak
10. Add info into swapfile : echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
11. Set swappiness to 10 : sudo sysctl vm.swappiness=10
12. Adjusting cache pressure setting : sudo sysctl vm.vfs_cache_pressure=50
13. Add info into sysctl.conf file at last line:
    (a) vm.swappiness=10
    (b) vm.vfs_cache_pressure=50
14. Back to catkin_dk folder : cd ~/catkin_dk
15. Make the file : catkin_make
16. Restart robot : sudo reboot

    -----BRINGUP THE ROBOT----
1. Type : ```roslaunch donkey_llc donkey_bringup.launch```
