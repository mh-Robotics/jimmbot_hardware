jimmbot_hardware
================

Robot ROS metapackage for the mhRobotics jimmBOT. Depended on the following packages:

 - jimmbot_hardware : jimmBOT hardware package (owns former base/sensors files).
 - jimmbot_description : jimmBOT robot description package. 
 - diff_drive_controller : Differential drive controller package used in jimmBOT.
 - joint_state_controller : Joint state controller package used in jimmBOT.
 - robot_state_publisher : Robot state publisher package used in jimmBOT.
 - controller_manager : Controller manager package used in jimmBOT.

## Udev rules
These udev rules are provided as a reference for some sensors typically installed on the jimmBOT.

To install them manually, execute:

```bash
sudo cp $(rospack find jimmbot_hardware)/udev/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
Or install them automatically, execute:

```bash
sh $(rospack find jimmbot_hardware)/install/udev-rules_focal_fossa.sh
```

Note: Until further solution, I filter Lidar using the Port Number in my robot computer. (I have exactly the same VID and PID with ESP32)