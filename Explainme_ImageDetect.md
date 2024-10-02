# Object Detection Node using ROS2 and OpenCV

This repository contains a ROS2 node implemented in Python that detects red, blue, and green spherical objects (like balls) from a camera feed. The detection is based on color segmentation and shape analysis using OpenCV.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Node](#running-the-node)
- [Script Explanation](#script-explanation)
  - [ROS2 Node Structure](#ros2-node-structure)
  - [Subscriptions and Publications](#subscriptions-and-publications)
  - [Image Processing Pipeline](#image-processing-pipeline)
    - [Color Space Conversion](#color-space-conversion)
    - [Color Thresholding](#color-thresholding)
    - [Morphological Operations](#morphological-operations)
    - [Contour Detection](#contour-detection)
    - [Circularity and Area Filtering](#circularity-and-area-filtering)
    - [Color Determination](#color-determination)
- [Theory and Techniques Used](#theory-and-techniques-used)
  - [OpenCV](#opencv)
  - [HSV Color Space](#hsv-color-space)
  - [Morphological Operations](#morphological-operations-1)
  - [Contour Analysis](#contour-analysis)
- [Why OpenCV](#why-opencv)
- [Conclusion](#conclusion)

## Introduction

This ROS2 node detects red, blue, and green balls in a camera image stream. It publishes whether an object is detected and the color of the detected object. The node leverages OpenCV for image processing due to its efficiency and simplicity, making it suitable for real-time applications with limited processing power.

## Prerequisites

- ROS2 Humble Hawksbill installed.
- Python 3.x.
- OpenCV library (`opencv-python`).
- `cv_bridge` package for converting ROS image messages to OpenCV images.
- A camera publishing images to the `/camera/color/image_raw` topic (adjustable in the script).

## Installation

1. Clone this repository into your ROS2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/object_detection_node.git
   ```

2. Install dependencies:

   ```bash
   pip install opencv-python numpy
   sudo apt install ros-humble-cv-bridge
   ```

3. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Running the Node

To run the object detection node, execute:

```bash
ros2 run object_detection_node object_detection_node.py
```

Ensure that the camera node is also running and publishing images to the specified topic.

## Script Explanation

### ROS2 Node Structure

The script defines a ROS2 node named `ObjectDetectionNode`, which inherits from `Node`. It initializes subscriptions and publishers, handles image processing in the callback, and publishes detection results.

### Subscriptions and Publications

- **Subscriptions:**
  - `/camera/color/image_raw`: Subscribes to the raw image topic from the camera.
  
- **Publishers:**
  - `/object_detected` (`Bool`): Publishes `True` if an object is detected, else `False`.
  - `/color_detected` (`String`): Publishes the color (`'red'`, `'blue'`, `'green'`) of the detected object.

### Image Processing Pipeline

#### Color Space Conversion

The received BGR image is converted to the HSV color space using:

```python
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
```

HSV is used because it separates color information (hue) from intensity, making color-based segmentation more effective.

#### Color Thresholding

Define HSV ranges for red, blue, and green colors:

```python
# Red has two ranges due to hue wrapping
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

# Blue and green ranges
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])
lower_green = np.array([35, 100, 100])
upper_green = np.array([85, 255, 255])
```

Create masks for each color:

```python
mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)

mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
mask_green = cv2.inRange(hsv, lower_green, upper_green)
```

#### Morphological Operations

Apply morphological operations to reduce noise:

```python
kernel = np.ones((5, 5), np.uint8)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_DILATE, kernel)
# Repeat for mask_blue and mask_green
```

- **Opening** removes small objects.
- **Dilation** enlarges the remaining objects.

#### Contour Detection

Combine all masks and find contours:

```python
combined_mask = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_blue), mask_green)
contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

#### Circularity and Area Filtering

Filter contours based on circularity and area to identify spherical objects:

```python
for contour in contours:
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    circularity = 4 * np.pi * (area / (perimeter * perimeter))
    if circularity > self.circularity_threshold and area > self.min_contour_area:
        # Proceed with color determination
```

- **Circularity** measures how close the shape is to a perfect circle.
- **Area Threshold** ensures the object is of significant size.

#### Color Determination

Determine which color mask overlaps most with the contour:

```python
overlap_red = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_contour))
overlap_blue = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_contour))
overlap_green = cv2.countNonZero(cv2.bitwise_and(mask_green, mask_contour))

max_overlap = max(overlap_red, overlap_blue, overlap_green)
if max_overlap == overlap_red:
    color_found = 'red'
elif max_overlap == overlap_blue:
    color_found = 'blue'
elif max_overlap == overlap_green:
    color_found = 'green'
```

Publish detection results if a color is found.

## Theory and Techniques Used

### OpenCV

OpenCV (Open Source Computer Vision Library) is an open-source computer vision and machine learning software library. It provides a common infrastructure for computer vision applications and accelerates the use of machine perception.

### HSV Color Space

The HSV (Hue, Saturation, Value) color space represents colors in terms of their shade (hue), vibrancy (saturation), and brightness (value). It is more aligned with human color perception and is better for color segmentation than the BGR or RGB spaces.

### Morphological Operations

Morphological operations process images based on shapes and are used for noise reduction and object isolation. The key operations used are:

- **Erosion**: Removes pixels on object boundaries.
- **Dilation**: Adds pixels to object boundaries.
- **Opening**: Erosion followed by dilation, useful for removing small objects.
- **Closing**: Dilation followed by erosion, useful for closing small holes.

### Contour Analysis

Contours are curves joining all continuous points along a boundary with the same color or intensity. Contour analysis helps in:

- Detecting object boundaries.
- Calculating shape properties like area, perimeter, and circularity.
- Filtering objects based on these properties.

## Why OpenCV

OpenCV is chosen for this task due to:

- **Efficiency**: Optimized for real-time applications.
- **Simplicity**: High-level functions for complex operations.
- **Community Support**: Extensive documentation and community support.
- **Lightweight**: Suitable for systems with limited processing power.

These features make OpenCV ideal for developing real-time object detection systems in robotics.

## Conclusion

This object detection node effectively identifies colored spherical objects using ROS2 and OpenCV. By leveraging color segmentation and shape analysis, it provides real-time detection suitable for robotic applications with limited resources.

Feel free to contribute to this project by opening issues or submitting pull requests.

---

**Note:** Adjust the HSV ranges, morphological kernel size, and area thresholds based on your specific environment and camera characteristics for optimal performance.