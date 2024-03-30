# DonkeyCar-ROS

DonkeyCar on a Raspberry Pi with ROS and Ubiquity.

## Software Requirements

On the Raspberry Pi, we use Ubiquity as the platform for images.

Download Raspberry Pi images Ubiquity from https://ubiquity-pi-image.sfo2.digitaloceanspaces.com/2023-02-09-ubiquity-base-gdm3-focal-raspberry-pi.img.xz.

## Installation Software on DonkeyCar

To get updates, turn off the Raspberry Pi hotspot and make an internet connection.

- Make sure your Debian package index is up-to-date:

```bash
sudo apt update
```
FYI, as soon as the Raspberry Pi connects to the internet, Ubiquity will continue to update in the background.

- Rebooting the Raspberry Pi is thus the best method to determine when the completion is complete:
```#
sudo reboot
```

- Then, install **samba**:
```#
sudo apt install samba
```

If it didn't work, try ```sudo reboot``` then ```sudo apt update``` again.

Go to `/etc/samba` and update the file name `smb.conf` with the following changes:
```#
#Windows Internet Name Serving Support Section:
#Wins Support - Tells the NMBD component of Samba to enable its WINS Server
    wins support = yes
```
and
```#
[catkin_dk_src]
    comment = Samba on Ubuntu
    path = /home/ubuntu/catkin_dk/src/
    read only = no
    browsable = yes
```

- Then, update Samba password and enter a new password whenever asked.`Prefer = Ubuntu`:
```#
sudo smbpasswd -a ubuntu
```


- Reboot Raspberry Pi:
```#
sudo reboot
```

- After that,install the libi2c library:
```#
sudo apt-get install libi2c-dev
```

Setting up the path for Roscore at this place and editing the file name ros_setup.bash:

```#
cd /etc/ubiquity
sudo nano ros_setup.bash
```

In the file text, `comment` the first catkin_setup line and enter **NEW** catkin_setup:
```#
catkin_setup=/home/ubuntu/catkin_dk/devel/setup.bash && test -f $catkin_setup && . $catkin_setup
```
- Reboot Raspberry Pi:
```#
sudo reboot
```

Create and make the `catkin`:
```#
mkdir -p ~/catkin_dk/src
cd ~/catkin_dk
catkin_make
```

## Setup ROS Master and ROS Hostname
Set the ros master and ros hostname on your Raspberry Pi.

- Add the three lines listed below to the `/.bashrc` file:
```#
source ~/catkin_dk/devel/setup.bash
export ROS_MASTER_URI=http://<robot_IP>:11311
export ROS_HOSTNAME=<robot_IP>
```

- Then, source the file:
```#
source ~/.bashrc
```

## Install ROS Servo Package
We use the `PCA9685` servo without the Adafruit library. We utilize ros2ic-pwmboard from [mentor-dyun](https://github.com/mentor-dyun/ros-i2cpwmboard.git).

Ros-i2cpwmboard is a project for the i2cpwm_board controller node.

### Installation
- Enter src folder and clone:
```#
cd ~/catkin_dk/src
git clone https://github.com/mentor-dyun/ros-i2cpwmboard.git
cd ~/catkin_dk
catkin_make
```
- Then, source the environment:
```#
source ~/.bashrc
```

## Create ROS Package

- Create new ros package for DonkeyCar:
```#
cd ~/catkin_dk/src
catkin_create_pkg donkey_llc rospy
```

- Create a Python script in `catkin_dk/src/donkey_llc/src` and name it `low_level_control.py`:
```#
#!/usr/bin/python3

"""
Class for low level control of our car. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time


class ServoConvert():
    def __init__(self, id=1, center_value=333, range=90, direction=1):
        self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range
        self._dir       = direction
        self.id         = id

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        self.value_out  = int(self._dir*value_in*self._half_range + self._center)
        print (self.id, self.value_out)
        return(self.value_out)

class DkLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('dk_llc')

        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=1)
        self.actuators['steering']  = ServoConvert(id=2, direction=1) #-- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg       = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 5

        rospy.loginfo("Initialization complete")

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.items():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        print (time.time() - self._last_time_cmd_rcv)
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print (self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    dk_llc     = DkLowLevelCtrl()
    dk_llc.run()
```
*copy from [tizianofiorenzani](https://github.com/tizianofiorenzani/ros_tutorials/blob/master/donkey_car/src/low_level_control.py).

- Then, make the file `low_level_control.py` executable:
```#
chmod +x low_level_control.py
```

## Install ROS Teleop_Twist_Keyboard
We utilize ros_teleop to control donkeycar movement.

- Install `ros_teleop` in `catkin_dk`:
```#
cd ~/catkin_dk/src
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
cd ..
catkin_make
```

- Then, source the environment:
```#
source ~/.bashrc
```

## Install Raspicam Node
We use the raspicam node from [UbiquityRobotics](https://github.com/UbiquityRobotics/raspicam_node).


------- FOR RASPBERRY PI 4 -------

- clone the file to `~/catkin_dk/src` and `catkin` it:
```#
cd ~/catkin_dk/src
git clone https://github.com/UbiquityRobotics/raspicam_node
cd ..
catkin_make
```
There are some dependencies that are not recognized by ros, so you need to create the file /etc/ros/rosdep/sources.list.d/ and create new file name `30-ubiquity.list`.

- Add this this to it:
```#
yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
```
- Then run `rosdep update`.

Install the ros dependencies:
```#
cd ~/catkin_dk
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y 
```

Compile the code with `catkin_make`.


------- FOR RASPBERRY Pi 3b+ -------

For the Raspberry Pi 3, we use [Dganbold's](https://github.com/dganbold/raspicam_node.git) raspicam node. In comparison to [Ubiquity](https://github.com/UbiquityRobotics/raspicam_node.git), this node is quite stable when running on a Raspberry Pi 3.

- clone the file to `~/catkin_dk/src`:
```#
cd ~/catkin_dk/src
git clone https://github.com/dganbold/raspicam_node.git
cd ..
```
Before running `catkin_make`, we need to increase the swap file size by 1 GB in order to reduce CPU utilization congestion.

- Allocate a 1GB swap area and **verify** it:
```#
sudo fallocate -l 1G /swapfile
ls -lh /swapfile Output : -rw-r--r-- 1 root root 1.0G /swapfile
```

- Enable swapfile and validate it.
```#
sudo chmod 600 /swapfile
ls -lh /swapfile Output : -rw------- 1 root root 1.0G /swapfile
```

- We now have a file with the correct permissions. To tell our system to format the file for swap, we can type:
```#
sudo mkswap /swapfile
```

- Now, tell the system it can use the swap file by typing:
```#
sudo swapon /swapfile
```

- Making swapfile permanent
```#
sudo cp /etc/fstab /etc/fstab.bak
```

- Add info into swapfile:
```#
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

- Set swappiness to 10:
```#
sudo sysctl vm.swappiness=10
```

- Adjusting cache pressure setting:
```# 
sudo sysctl vm.vfs_cache_pressure=50
```

- Add info into `sysctl.conf` file at last line:
```#
    vm.swappiness=10
    vm.vfs_cache_pressure=50
```

Then, compile `~/catkin_dk` with `catkin_make`.

## BRINGUP DonkeyCar

To bring up donkeycar, we must build a `launch` file in `catkin_dk`.

Create `.launch` file:
```#
mkdir ~/catkin_dk/src/donkey_llc/launch
cd ~/catkin_dk/src/donkey_llc/launch/
sudo nano donkey_bringup.launch
```

Create script in file `donkey_bringup.launch`:
```#
<launch>
  
<include file="$(find i2cpwm_board)/launch/i2cpwm_node.launch"/>
  
<include file="$(find raspicam_node)/launch/camerav2_410x308_30fps.launch">
<arg name="enable_raw" value="true"/>
</include>
  
<node pkg="donkey_llc" name="dk_llc" type="low_level_control.py" output="screen">
</node>

</launch>
```
To bringup the donkeycar, type:
```#
roslaunch donkey_llc donkey_bringup.launch
```
