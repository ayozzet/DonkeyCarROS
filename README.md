For Robot Preparation:

1. Download Raspberry Pi image Ubiquity from : https://ubiquity-pi-image.sfo2.digitaloceanspaces.com/2023-02-09-ubiquity-base-gdm3-focal-raspberry-pi.img.xz
2. Stop the RPi hostspot and connect to the internet for update.
3. Update the RPi image : sudo apt update
4. FYI, Ubiquity will automatically update once RPi connected to the internet.
5. Install samba : sudo apt install samba
6. Go to /etc/samba and edit file name smb.conf with below addition:
   a) wins support = yes
   b) [catkin_dk_src]
         comment = Samba on Ubuntu
         path = /home/ubuntu/catkin_dk/src
         read only = no
         browsable = yes
8. Update Samba password : sudo smbpasswd -a ubuntu
9. Enter new password whenever ask.
10. 
