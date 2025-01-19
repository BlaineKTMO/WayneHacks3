# WayneHacks 3

Mono repo containing project files for WayneHacks 3 😄

## Overview

This project integrates data from a 360-degree camera and a Velodyne LiDAR sensor to create a fused image. The fused image combines visual data from the camera with depth information from the LiDAR, providing a comprehensive view of the environment.

## Running Camera LiDAR Fusion

### Prerequisites:

- Requires a 360 Camera and 3D LiDAR.
- ROS 2 packages are built for Ubuntu-22.04 and ROS 2 Humble.

### Steps:

1. Clone the repository
   ```
   git clone https://github.com/BlaineKTMO/WayneHacks3.git
   ```
2. Navigate to the fusion workspace, install dependencies, and build
   ```
   cd WayneHacks3/360_Cam_Lidar_Fusion/
   pip install -r requirements.txt
   rosdep install --from-paths src -y --ignore-src
   colcon build && source install/setup.bash
   ```
3. Run the launch file
   ```
   ros2 launch fusion full_fusion.launch.py video_source:=2
   ```

## Running Lane Segmentation

### Prerequisites

- Requires a camera
- Running Ubuntu-22.04 with ROS 2 Humble

### Steps:

1. Navigate to the fusion workspace, install dependencies, and build
   ```
   cd WayneHacks3/360_Cam_Lidar_Fusion/
   pip install -r requirements.txt
   rosdep install --from-paths src -y --ignore-src
   colcon build && source install/setup.bash
   ```
2. Identify your video source
   ```
   ls /dev/video*
   ```
3. Run the image publisher
   ```
   ros2 run fusion img_pub --ros_args -p video_source:=<Webcam video source>
   ```
4. In a seperate terminal, run the lane segmenter
   ```
   ros2 run fusion line_follow
   ```

## Running the Unity Visualization

### Prerequisites

- Requires a VR Headset
- Running Unity 2022 on Windows 11

### Steps

1. Install necessary drivers for VR headset development for your specific headset.
2. Create a VR Unity project.
3. Create a sky box object with an attached video player.
4. Add your recorded mp4 of equirectangular image as an asset.
5. Drag the video asset into the video player video source.
6. Run the game.

## Hardware

- This project utilized an Insta 360 X4, Velodyne VLP-16, and a LinkStar router running OpenWRT.
- The Insta 360 X4 was connected through USB and placed in Webcam mode.
- The VLP-16 served information through the LinkStar Router.
- Devices were interconnected through a LAN/WLAN hosted on the LinkStar Router.
- LAN/WLAN allowed us to share data across devices to promote network

## Sections

The repo is split into 4 sections:

- **360_Cam_Lidar_Fusion**: ROS 2 workspace containing packages for the visual sensor fusion.
  - Contains the Velodyne drivers and Fusion package.
  - Reads Insta 360 X4 and Velodyne VLP-16 data.
  - Fuses data by transforming the point cloud data onto the image using OpenCV.
  - Hardware acceleration enabled using CuPy and Cuda 11.5.
  - Lane segmentation using OpenCV and applying an HSV filter.
- **Lidar Interpolation**: ROS2 workspace containing a package for interpolating LiDAR data.
  - Increased resolution of LiDAR data from 16 layers to 32 layers.
  - Simple median interpolation.
  - Filtered based on a distance threshold to prevent obstacle hallucinations.
- **Website**: Simple website for hosting video stream of fused data.
  - Tried multiple approaches using VLC, FFmpeg, Video.js
  - Tried streaming over http, rtsp
  - Was able to get streams across computers working, but struggled piping ROS 2 image data into stream.
- **Unity**: Utilized Unity to visualize fused data to get a clearer picture of data.
  - Connected to Meta Quest VR Headset.
  - Used the Unity Video Player to visualize recorded MP4 data.
  - Projected MP4 data onto SkyBox object to give a 3D view.

## Features

- **360 Image and LiDAR Data Fusion**: Combines data from a 360-degree camera and a Velodyne LiDAR sensor.
- **Hardware Acceleration**: Utilizes GPU acceleration with CuPy for efficient processing.
- **Real-Time Visualization**: Displays the fused image in real-time using OpenCV.
- **Lane Segmentation**: Applies HSV filtering to collect lane data.
- **VR Vizualization**: It can be a little bit difficult to understand the project by looking at an equirectangular projection of 360-degree data, so the VR headset is used for demonstration.

## Struggles

- We spent a lot of time messing around with various applications, transport protocols, and data formats for streaming videos over web.
- In the end, we got stuck on streaming video from a ROS 2 message type and pivoted away.
- The fusion algorithm required the use of a transformation matrix from the LiDAR to the Camera. This was fine for when we mounted the Camera right above the LiDAR but struggled when holding the LiDAR tilted 45-degrees about the y-axis which was required for processing lane data.
- Underestimated the amount of processing power required for fusion. Even with a distributed system we saw latency between the LiDAR and the Camera.

## Celebrations

- We got a much better result from the fusion algorithm then expected allowing us to get a very detailed view of the environment.
- Successfully integrated Meta Quest to playback video feeds to get a "Robot Perspective"
- LiDAR interpolation algorithm gave us a higher resolution point cloud with scalability, meaning we are able to interpolate again and again until we balanced accuracy with resolution.
- Learned an incredible amount of video encoding, codec types, transport methods, and video data formats
- Found new functionalities within VLC Media Player and FFmpeg that could prove useful in future projects.
- Gained a deeper understanding on how 3D LiDARs work, 3D point cloud processing, and point cloud representations.
