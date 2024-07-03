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

We employed an IR array paired with a PID controller to ensure precise tracking of the line. The IR sensors detect the contrast between the white line and the black surface, allowing the PID controller to adjust the robot's trajectory dynamically.

**Technologies Used:**
- **IR Sensors:** Detect the contrast between the white line and black surface.
- **PID Controller:** Adjusts the robot's path dynamically based on sensor input.

### Wall Color Detection

We used OpenCV with computer vision from a web camera to identify wall colors. This setup helps maintain a consistent distance from the walls while detecting required colors for navigation.

**Technologies Used:**
- **OpenCV:** Processes images to detect wall colors.
- **Web Camera:** Captures real-time images for processing.

### Color Junction Detection

We strategically placed under-mounted color sensors to detect color junctions effectively. These sensors are fine-tuned to recognize specific color configurations at junctions, guiding the robot to navigate turns accurately.

**Technologies Used:**
- **Color Sensors:** Detect specific color configurations at junctions.

### 3D Object Detection

We used TensorFlow and OpenCV to identify the shape of objects, determining whether an object is a cylinder or a cube. This combination of technologies provides robust shape recognition capabilities.

**Technologies Used:**
- **TensorFlow:** Machine learning framework for object detection.
- **OpenCV:** Processes images to identify object shapes.

### Metal Box Detection and Grabbing

We integrated conductivity testing to identify metal boxes and designed a robotic arm for precise grabbing. The conductivity sensor differentiates between metal and non-metal boxes, while the robotic arm handles the pickup and placement tasks efficiently.

**Technologies Used:**
- **Conductivity Sensor:** Differentiates between metal and non-metal boxes.
- **Robotic Arm:** Handles the precise grabbing and placement of boxes.

### Obstacle Height Detection

Three ToF (Time-of-Flight) sensors were utilized to measure the heights of obstacles accurately. This data helps the robot calculate the total number of gems collected based on obstacle heights.

**Technologies Used:**
- **ToF Sensors:** Measure the height of obstacles accurately.

### Execution Methodology
Each challenge was tackled using sophisticated sensors and control algorithms:

- **Line Following:** Employed IR array and PID controller for precise path adjustment.
- **Wall Color Detection:** Used OpenCV with computer vision from a web camera to identify wall colors.
- **Color Junction Detection:** Implemented under-mounted color sensors for precise junction recognition.
- **3D Object Detection:** Used TensorFlow and OpenCV to identify shapes, distinguishing between cylinders and cubes.
- **Metal Box Detection and Grabbing:** Conductivity testing and robotic arm for efficient handling.
- **Obstacle Height Detection:** ToF and ultrasonic sensors for precise height measurement.
