# Position Ground Truth System for Round Robots using LiDARs

This repository contains the implementation of a high-accuracy ground truth system designed to detect and track round robots in indoor environments. Developed primarily for the RoboCup Small Size League (SSL), the system utilizes two 2D LiDARs and a scan-matching software architecture to provide a reliable alternative to overhead camera systems.

## 📌 Project Overview

In robotics research, evaluating localization systems requires a "ground truth"—a factual reference for the robot's position. While SSL-Vision is commonly used, it can suffer from noise, light interference, and fish-eye distortion.

This system achieves a mean error of less than 1 cm by merging data from two LiDARs (RPLIDAR A1 and S1). It processes laser points into a binary image and applies the Hough Transform to recognize the circular geometry of the robots.

## 🛠 Software Architecture

The software is implemented as a ROS2 node (object_detection_node.py) with the following pipeline:

1. **Scan Matching**: Finds the transformation (translation, rotation, and scale) to align the point clouds from both sensors.
2. **Data Merging**: Combines the synchronized laser data into a single 1000x1000 pixel global image mask.
3. **Morphological Processing**: * Dilation & Erosion: Connects nearby points to reinforce the robot's shape.
   - **Gaussian Blur**: Reduces noise and false positives.
4. **Robot Detection**: Uses the Hough Circles Transform (cv2.HoughCircles) to identify the robot's center and radius in the image.
5. **Coordinate Transformation**: Converts image-space detection back into real-world coordinates ($x, y$ in meters).

## 🚀 Key Features

- **Dual-LiDAR Support**: Specifically tuned for RPLIDAR A1 (triangulation) and S1 (ToF).
- **High Precision**: Demonstrated mean error of ~0.5cm in X and Y axes.
- **ROS2 Integrated**: Built for Ubuntu 22.04 and ROS2 (Humble/Foxy).
- **Automatic Logging**: Saves detection coordinates and timestamps to .txt files for post-processing and comparison.7

## 📋 Requirements

- ROS2 (Tested on Ubuntu 22.04)
- Python 3.x
- OpenCV (cv2)
- NumPy
- SciPy (for minimize optimization in scan matching)
- SLAMTEC RPLIDAR ROS2 Package

## Tests and results

More infos about the hardware and software architectures, methodology, the tests made and the results achieved can be found in the [paper](https://ieeexplore.ieee.org/abstract/document/11066123).

## Citation

> J. V. L. Aguiar and F. Tonidandel, "Position Ground Truth For a Round Robot Using LiDARs," 2025 Brazilian Conference on Robotics (CROS), Belo Horizonte, Brazil, 2025, pp. 1-6, doi: 10.1109/CROS66186.2025.11066123. keywords: {Location awareness;Laser radar;Accuracy;Robot kinematics;Machine vision;Robot vision systems;Measurement uncertainty;Cameras;Robots;Standards;Ground Truth;Position;LiDAR;Round Robots},
