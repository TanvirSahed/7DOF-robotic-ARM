# 7-DOF Robotic Arm: Object Classification & Sorting

A 7-DOF robotic arm (MG996R servos, ATmega2560, PCA9685 driver) that detects and sorts
objects (cups vs. bottles) using MobileNet-SSDV3 for computer vision, controlled through
**feedback position mapping** instead of kinematic equations.

Published as:

> T. R. Sahed, S. R. Islam, M. T. I. Aronno, M. S. R. Chowdhury, "Object Classification by a
> 7-DOF Robotic Arm Using MobileNet-SSD and Feedback Position Mapping," 2025 International
> Conference on Quantum Photonics, Artificial Intelligence, and Networking (QPAIN), Rangpur,
> Bangladesh. DOI: [10.1109/QPAIN66474.2025.11171666](https://doi.org/10.1109/QPAIN66474.2025.11171666)

- 🎥 Demo video: https://youtube.com/shorts/5LcHwnB7o1Q
- 🧩 3D design (Fusion 360): https://a360.co/46bxGCI

![Object detection: bottle](bottle.JPG)
![Object detection: cup](cup.JPG)

## How it works

- A WiFi smartphone camera streams video (IP Webcam) to a host PC running MobileNet-SSDV3
  (COCO-pretrained) to detect and classify cups/bottles in real time.
- **Feedback position mapping**: an extra feedback wire on each MG996R servo lets the
  microcontroller read its live angle. Target poses are recorded once by hand-positioning the
  arm, then replayed for pick-and-place, no kinematic equations needed.
- A PCA9685 driver regulates 6V/1.5A per channel across the 7 servos. The ATmega2560 receives
  the detected object's class over serial and drives a tuned PD controller (Kp=2.2, Kd=0.7)
  for smooth, low-jitter motion.

## Repository contents

| Path | What it is |
|---|---|
| `1RoboticArmFinal.ino` | Main Arduino Mega firmware: servo control, serial command handling, PD controller |
| `7DoFarmPositionMapping.ino` | Position-mapping sketch used to record joint angles for each pose |
| `RobotVision.py`, `RobotVision7Dof.py` | Host-side Python: MobileNet-SSD object detection + serial link to the Arduino |
| `arduinoCOM.py` | Serial communication helper |
| `coco.names.txt`, `frozen_inference_graph.pb` | MobileNet-SSD model/labels used for detection |
| `Object_Detection_Files.zip` | Bundled model config files |
| `ros2_ws/` | ROS 2 (Humble) URDF/xacro description of the arm, generated from the Fusion 360 CAD model, for RViz2 visualization and Gazebo simulation. See [`ros2_ws/src/7dof_description/README.md`](ros2_ws/src/7dof_description/README.md) |
| `bottle.JPG`, `cup.JPG`, `bottle_cup.JPG`, `bottle_arduino.JPG` | Detection sample images |

## Getting started

### Hardware / firmware

Flash `1RoboticArmFinal.ino` to an Arduino Mega with the servos wired per the pin mapping in
the sketch, then run the vision pipeline on the host PC:

```bash
python RobotVision7Dof.py
```

### ROS 2 visualization (no hardware required)

```bash
cd ros2_ws
colcon build --packages-select 7dof_description
source install/setup.bash
ros2 launch 7dof_description display.launch.py   # RViz2, with joint sliders
ros2 launch 7dof_description gazebo.launch.py     # Gazebo simulation
```

See [`ros2_ws/src/7dof_description/README.md`](ros2_ws/src/7dof_description/README.md) for
requirements and details.

## Results

- 81.25% object-detection accuracy, 90% sorting accuracy
- ~2-second sort cycle, 9/10 successful independent trials
- Sub-centimeter pose repeatability (mean error 0.53 cm, σ 0.44 cm over a 1,000-trial Monte Carlo simulation)
- 1.42 kg payload capacity

## Author

Md Tanvir Rahman Sahed, tanvir.sahed00@gmail.com
