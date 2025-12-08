### วิธีใช้งาน

```bash
sudo apt update
sudo apt install python3-pynput
```

```bash
# Download the repository to your machine
git clone https://github.com/fibo-github-classroom/multiverse-mission-pond-ply.git
cd multiverse-mission-pond-ply/
```

```bash
# Build and source work space
colcon build
colcon build --packages-select turtlesim_plus turtlesim_plus_interfaces
source install/setup.bash
```

```bash
# Run all file
ros2 launch Exam1 exam1_bringup.launch.py 
```

### Call Services

```bash
# Set max pizza for teleop turtle
ros2 service call /teleop/max_pizza controller_interface/srv/SetMaxpizza "max_pizza:
  data: 20" 
```

```bash
# Set param for teleop turtle
ros2 service call /teleop/set_param controller_interface/srv/SetParam "kp_linear:
  data: 2.0
kp_angular:
  data: 5.0"
```

```bash
# Set param for copy turtle
ros2 service call /copy/set_param controller_interface/srv/SetParam "kp_linear:
  data: 2.0
kp_angular:
  data: 10.0"
```

```bash
# Set param for eraser turtle
ros2 service call /eraser/set_param controller_interface/srv/SetParam "kp_linear:
  data: 2.0
kp_angular:
  data: 10.0"
```
