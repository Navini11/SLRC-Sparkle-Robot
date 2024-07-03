# SLRC-Sparkle-Robot

## About SLRC
The Sri Lankan Robotics Challenge (SLRC) is a distinguished event hosted by the Department of Electronic and Telecommunication Engineering at the University of Moratuwa. The competition is divided into two segments: the School Category and the University Category.

## The Team
We are a group of five enthusiastic undergraduates from the Department of Electronic and Telecommunication Engineering at the University of Moratuwa, collectively known as "Sparkle." Our team members include:

- Pulindu Vidmal
- Akhila Prabodha
- Achira Hansindu
- Navini Jagoda
- Devnith Wijesinghe

Our team secured a commendable top 15 rank in the university category and reached the finals of the competition.

## Our Task
Our project was inspired by the theme of infinity stones from the Avengers movie series. For a detailed explanation of our tasks, please refer to the attached PDF document.

[Download Task PDF](https://ent.uom.lk/wp-content/uploads/2024/01/SLRC-2024-University-Category-v1.pdf)

## Project Challenges
Our tasks were segmented into three thematic regions, each inspired by different celestial settings:

1. **Planet A - Mountains of Vormir**
2. **Planet B - Ruins of Sakaar**
3. **Planet C - Thanos’s Home**

## Task Breakdown

### Line Following

We implemented an IR array combined with a PID controller to ensure accurate line tracking. The IR sensors detect the contrast between the white line and the black surface, allowing the PID controller to make real-time adjustments to the robot's path.

### Wall Color Detection

Using OpenCV and computer vision with a web camera, we identified wall colors. This system helps maintain a consistent distance from the walls while detecting necessary colors for navigation.

### Color Junction Detection

Strategically placed under-mounted color sensors were used to detect color junctions. These sensors are calibrated to recognize specific color patterns at junctions, guiding the robot through turns accurately.

### 3D Object Detection

We utilized TensorFlow and OpenCV to identify object shapes, distinguishing between cylinders and cubes. This technology combination provides robust shape recognition capabilities.

### Metal Box Detection and Grabbing

Conductivity testing was employed to identify metal boxes, and a robotic arm was designed for precise grabbing. The conductivity sensor differentiates between metal and non-metal boxes, while the robotic arm handles pickup and placement tasks efficiently.

### Obstacle Height Detection

Three ToF sensors were used to measure obstacle heights accurately. This data helps the robot calculate the total number of gems collected based on obstacle heights.

### Execution Methodology
Each challenge was tackled using sophisticated sensors and control algorithms:

- **Line Following:** Employed IR array and PID controller for precise path adjustment.
- **Wall Color Detection:** Used OpenCV with computer vision from a web camera to identify wall colors.
- **Color Junction Detection:** Implemented under-mounted color sensors for precise junction recognition.
- **3D Object Detection:** Used TensorFlow and OpenCV to identify shapes, distinguishing between cylinders and cubes.
- **Metal Box Detection and Grabbing:** Conductivity testing and robotic arm for efficient handling.
- **Obstacle Height Detection:** ToF and ultrasonic sensors for precise height measurement.
