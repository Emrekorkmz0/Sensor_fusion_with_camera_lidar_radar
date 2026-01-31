# ScanTo3DPCD: 3D Point Cloud and Object Detection Using LIDAR and Camera Data

This project utilizes a LIDAR sensor and a camera to detect objects in the environment, convert the data into a 3D point cloud, and perform object detection, saving the coordinates of the detected objects. Developed with ROS2 (Robot Operating System 2), this project is ideal for use in autonomous systems, robotics, and IoT applications.

## Features

- **3D Point Cloud from LIDAR Data**: Data from the LIDAR sensor is projected into 3D points, with the Z-axis modulated using a sine function, allowing the point cloud to dynamically change with each scan.
- **Object Detection with Camera Data**: Using the YOLO (You Only Look Once) model, objects are detected in camera images. The detected objects are then matched with LIDAR data points and visualized together.
- **Radar Motion Detection**: Motion detection is performed using an Arduino-based radar sensor. When motion is detected, false points (black dots) are added to the 3D point cloud.
- **Visual Outputs**: The LIDAR data and YOLO object detection results are saved as PNG images and visualized.
- **File Saving**: The output includes a `.ply` point cloud file and a text file containing object detection results.

## How It Works

1. **LIDAR Scanning**: LIDAR data is obtained from the `/scan` topic and projected into 3D points using the `LaserProjection` class.
2. **Image Processing**: Camera data from the `/camera/image_raw` topic is processed using the YOLO model to detect objects.
3. **Matching and Detection**: LIDAR points are matched with objects detected in the camera image. The detected objects' information is saved.
4. **Motion Detection**: The radar sensor detects motion. When motion is detected, additional points are added to the 3D point cloud.
5. **Saving Results**: The point cloud is saved as a `.ply` file. Detected objects and their color information are saved to a text file.
6. **Visualization**: The point cloud is projected to 2D and saved as a PNG image.

## Requirements

- Python 3.6+
- ROS2 (Foxy or later)
- YOLOv11 model file
- OpenCV
- PyntCloud
- Pandas, NumPy, Matplotlib
- Arduino for serial connection (optional)
