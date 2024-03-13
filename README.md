For Robot Preparation:

1. Download Raspberry Pi image Ubiquity from : https://ubiquity-pi-image.sfo2.digitaloceanspaces.com/2023-02-09-ubiquity-base-gdm3-focal-raspberry-pi.img.xz
2. Stop the RPi hostspot and connect to the internet for update.
3. Update the RPi image : sudo apt update
4. FYI, Ubiquity will continue update in background once RPi connected to the internet.
5. So, the best way to know the completion is completed by rebooting the RPi : sudo reboot
6. Install samba : sudo apt install samba
7. Run again Step #6 if it failed.
8. Go to /etc/samba and edit file name smb.conf with below addition:
   (a) wins support = yes
   (b) [catkin_dk_src]
         comment = Samba on Ubuntu
         path = /home/ubuntu/catkin_dk/src
         read only = no
         browsable = yes
9. Update Samba password : sudo smbpasswd -a ubuntu
10. Enter new password whenever ask. (Prefer = ubuntu)
11. Restart robot : sudo reboot
12. Install I2C library : sudo apt-get install libi2c-dev
13. Setting up the path for roscore at this location : cd /etc/ubiquity/
14. Edit file name ros_setup.bash : sudo nano ros_setup.bash
15. Comment the 1st catkin_setup line and add NEW catkin_setup :catkin_setup=/home/ubuntu/catkin_dk/devel/setup.bash && test -f $catkin_setup && . $catkin_setup
16. Restart robot : sudo reboot
17. Create catkin : mkdir -p ~/catkin_dk/src
18. Go to catkin folder : cd catkin_dk
19. Make the catkin : catkin_make
20. Add 3 lines as below in .bashrc file
    (a) source ~/catkin_dk/devel/setup.bash
    (b) export ROS_MASTER_URI=http://<robot_IP>:11311
    (c) export ROS_HOSTNAME=<robot_IP>
22. Load the new setup : source ~/.bashrc
23. Go to src folder : cd ~/catkin_dk/src
24. Git clone file : git clone https://github.com/mentor-dyun/ros-i2cpwmboard.git
25. Back to catkin folder : cd ..
26. Make the catkin : catkin_make
27. Load the new setup : source ~/.bashrc
28. Create ROS package in catkin_dk/src folder : catkin_create_pkg donkey_llc rospy
29. Create Python script inside catkin_dk/src/donkey_llc/src and name as low_level_control.py <get the the copy in Tiziano github
30. Make the low_level_control.py executable : chmod +x low_level_control.py
31. Back to catkin/src folder : cd ~/catkin_dk/src
32. Git clone file : git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
33. Back to catkin_dk folder : cd ~/catkin_dk
34. Make the catkin : catkin_make
35. Back to catkin_dk/src folder : cd ~/catkin_dk/src
36. Git clone file : git clone https://github.com/UbiquityRobotics/raspicam_node.git
37. Go to /etc/ros/rosdep/sources.list.d/ folder.
38. Create a file name as 30-ubiquity.list
39. Add this line into that file : yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
40. Update ROS repositoty : rosdep update
41. Back to catkin_dk folder : cd ~/catkin_dk
42. Run this code : rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
43. Make the file : catkin_make
44. For RPi3B+ USE this command :  catkin_make -j2
