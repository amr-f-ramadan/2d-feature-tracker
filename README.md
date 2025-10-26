# 2D Feature Tracking

> **Note**: This project was originally submitted as part of Udacity's Sensor Fusion Nanodegree in 2020. It has been published on GitHub with improvements and refinements.

![Skills](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-27338e?style=flat&logo=OpenCV&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-064F8C?style=flat&logo=cmake&logoColor=white)

**Tags**: `computer-vision` `feature-detection` `feature-matching` `keypoint-detection` `SIFT` `SURF` `FAST` `BRISK` `ORB` `AKAZE` `HARRIS` `descriptor-matching` `2d-tracking` `autonomous-vehicles` `sensor-fusion` `FLANN` `KNN-matching`

<img src="images/keypoints.png" width="820" height="248" />

## Project Overview

This project implements a comprehensive 2D feature tracking system for autonomous vehicle applications. I built and benchmarked various combinations of keypoint detectors and feature descriptors to identify the most efficient approaches for real-time collision detection systems.

### My Contributions

I implemented and evaluated multiple components of the feature tracking pipeline:

* **Data Structure Management**: Designed a ring buffer system to efficiently manage image sequences and optimize memory usage
* **Keypoint Detection**: Integrated and compared 7 different detectors (HARRIS, FAST, BRISK, ORB, AKAZE, SIFT, and Shi-Tomasi)
* **Feature Description**: Implemented 6 descriptor algorithms (BRISK, BRIEF, ORB, FREAK, AKAZE, and SIFT)
* **Matching Algorithms**: Developed both Brute Force and FLANN-based matching with KNN selection and distance ratio filtering
* **Performance Analysis**: Created a comprehensive benchmarking system (`experiments.py`) to evaluate 42 detector/descriptor combinations
* **Optimization**: Identified the top 3 detector/descriptor pairs based on speed and accuracy metrics

## Project Structure

```
2d-feature-tracker/
├── CMakeLists.txt              # Build configuration
├── experiments.py              # Automated benchmarking script
├── LICENSE                     # Project license
├── README.md                   # This file
├── images/                     # Image assets
│   └── KITTI/                  # KITTI dataset images
│       └── 2011_09_26/
│           └── image_00/
│               └── data/
├── output/                     # Results and analysis
│   └── result.txt              # Benchmark results
└── src/                        # Source code
    ├── dataStructures.h        # Data structure definitions
    ├── matching2D_Student.cpp  # Feature detection & matching implementation
    ├── matching2D.hpp          # Header file
    └── MidTermProject_Camera_Student.cpp  # Main application
```

## Technical Implementation

### Keypoint Detection Methods
I implemented the following detection algorithms:
- **Shi-Tomasi**: Good features to track corner detector
- **HARRIS**: Classic corner detection algorithm
- **FAST**: Features from Accelerated Segment Test
- **BRISK**: Binary Robust Invariant Scalable Keypoints
- **ORB**: Oriented FAST and Rotated BRIEF
- **AKAZE**: Accelerated-KAZE features
- **SIFT**: Scale-Invariant Feature Transform

### Feature Descriptors
- **BRISK**: Binary descriptor
- **BRIEF**: Binary Robust Independent Elementary Features
- **ORB**: Oriented BRIEF
- **FREAK**: Fast Retina Keypoint
- **AKAZE**: AKAZE descriptor (only works with AKAZE detector)
- **SIFT**: SIFT descriptor

### Matching Strategy
- Implemented KNN matching with k=2
- Applied descriptor distance ratio filtering with threshold t=0.8
- Supported both Brute Force and FLANN-based matchers
- Handled binary and HOG-based descriptors appropriately


* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

### Matching Strategy
- Implemented KNN matching with k=2
- Applied descriptor distance ratio filtering with threshold t=0.8
- Supported both Brute Force and FLANN-based matchers
- Handled binary and HOG-based descriptors appropriately

## Performance Benchmark Results

I created an automated benchmarking system that evaluates all possible detector-descriptor combinations. The matching example is shown below:

<img src="images/example.png" width="820" height="180" />

### Running the Benchmark

Run the automated benchmark: `python experiments.py`

This generates a comprehensive analysis in `output/result.txt`.

### Complete Results Table

I used KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8:

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | SHITOMASI + BRISK |1179 |767 |2291.16 |
|2 | SHITOMASI + BRIEF |1179 |944 |125.191 |
|3 | SHITOMASI + ORB |1179 |908 |126.824 |
|4 | SHITOMASI + FREAK |1179 |768 |365.967 |
|5 | SHITOMASI + AKAZE |N/A |N/A |N/A |
|6 | SHITOMASI + SIFT |1179 |788 |188.365 |
|7 | HARRIS + BRISK |248 |142 |2276.44 |
|8 | HARRIS + BRIEF |248 |173 |135.613 |
|9 | HARRIS + ORB |248 |162 |139.757 |
|10 | HARRIS + FREAK |248 |144 |376.209 |
|11 | HARRIS + AKAZE |N/A |N/A |N/A |
|12 | HARRIS + SIFT |248 |166 |212.742 |
|13 | FAST + BRISK |1491 |899 |2190.39 |
|14 | FAST + BRIEF |1491 |1099 |48.353 |
|15 | FAST + ORB |1491 |1071 |47.8879 |
|16 | FAST + FREAK |1491 |878 |310.866 |
|17 | FAST + AKAZE |N/A |N/A |N/A |
|18 | FAST + SIFT |1491 |924 |115.162 |
|19 | BRISK + BRISK |2762 |1471 |4581.71 |
|20 | BRISK + BRIEF |2762 |1704 |2416.18 |
|21 | BRISK + ORB |2762 |1675 |2421.21 |
|22 | BRISK + FREAK |2762 |1460 |2691.6 |
|23 | BRISK + AKAZE |N/A |N/A |N/A |
|24 | BRISK + SIFT |2762 |1361 |2495.69 |
|25 | ORB + BRISK |1161 |553 |2327.52 |
|26 | ORB + BRIEF |1161 |545 |176.963 |
|27 | ORB + ORB |1161 |562 |184.929 |
|28 | ORB + FREAK |1161 |512 |446.59 |
|29 | ORB + AKAZE |N/A |N/A |N/A |
|30 | ORB + SIFT |1161 |510 |245.217 |
|31 | AKAZE + BRISK |1670 |1193 |2556.14 |
|32 | AKAZE + BRIEF |1670 |1266 |436.603 |
|33 | AKAZE + ORB |1670 |1260 |425.875 |
|34 | AKAZE + FREAK |1670 |1178 |702.261 |
|35 | AKAZE + AKAZE |N/A |N/A |N/A |
|36 | AKAZE + SIFT |1670 |1082 |468.205 |
|37 | SIFT + BRISK |1387 |565 |2560.5 |
|38 | SIFT + BRIEF |1387 |704 |617.944 |
|39 | SIFT + ORB |1387 |683 |602.68 |
|40 | SIFT + FREAK |1387 |591 |858.346 |
|41 | SIFT + AKAZE |N/A |N/A |N/A |
|42 | SIFT + SIFT |1387 |625 |654.526 |

|41 | SIFT + AKAZE |N/A |N/A |N/A |
|42 | SIFT + SIFT |1387 |625 |654.526 |

### Top 3 Detector/Descriptor Combinations

Based on my analysis, I identified the best 3 detector/descriptor pairs according to total processing time while maintaining high match quality:

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | FAST + ORB |1491 |1071 |47.8879 |
|2 | FAST + BRIEF |1491 |1099 |48.353 |
|3 | FAST + SIFT |1491 |924 |115.162 |

**Key Findings**:
- FAST detector consistently provides the best performance in terms of speed
- ORB and BRIEF descriptors offer excellent speed with minimal quality loss
- The FAST+ORB combination achieves sub-50ms processing time, making it ideal for real-time applications

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Skills Demonstrated

- **Computer Vision**: Feature detection and tracking algorithms
- **C++ Programming**: Object-oriented design and STL usage
- **OpenCV**: Advanced computer vision library usage
- **Performance Optimization**: Benchmarking and algorithm comparison
- **Data Structures**: Ring buffer implementation for memory efficiency
- **Algorithm Analysis**: Quantitative comparison of detection/description methods

## Acknowledgments

This project was completed as part of Udacity's Sensor Fusion Engineer Nanodegree program.
|3 | SHITOMASI + BRIEF |1179 |944 |125.191 |
