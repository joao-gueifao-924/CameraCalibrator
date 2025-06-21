# CameraCalibrator

A desktop application using Qt 6, QML, and OpenCV to calibrate a single rectilinear camera using a chessboard pattern. It follows the pinhole camera model.

It features two main modes of execution:
  - **live camera feed**: where user can hold the pattern in front of the camera, one place at a time, with real time visual feedback (available, but refinements are still due)
  - **folder with image files**: where user provides a set of pre-recorded images of the chessboard appearing in multiple places (not yet available, work still in progress)

When calibration is finished, user can save the camera parameters into a JSON file for posterior use.

## Key Features

*   Live camera feed display from connected cameras
*   Interactive, real-time detection of chessboard patterns
*   Visualization of detected corners on the video stream
*   Calculation of camera intrinsics matrix and lens distortion coefficients

## Notes
- Development of this program is still in progress.
- Apart from Qt 6, you will also need QtMultimedia package installed. It is used for handling video camera feeds.
- This is a CMAKE project, with the following dependencies: OpenCV 4.x, Boost 1.85+
