This is an example implementation of a LUT based segmentation module that can be used with Realsense cameras.

Designed to use Python 3. Dependencies: numpy, openCV, pyrealsense2

This code is heavily inspired by [Mitupead - football robot](https://github.com/lwd8cmd/Mitupead)

## Modules in the code

### Color.py
Contains data of the colors that are detected. Also contains data used to display the colors in the debug view.

### camera.py
Contains a ICamera interface that describes methods required by the image processing module. Also contains an example implementation for Intel RealSense cameras (RealsenseCamera) and OpenCV web cameras (OpenCVCamera).

### image_processor.py 
Main module for image processing. Is responsible for image color and feature segmentation. Object analysis and filtration should happen in this module.
```
def process_frame(self, aligned_depth = False) -> ProcessedResults: 
```
Is the main method responsible for providing processed frame data.

### motion.py
Contains a IRobotMotion interface that descibes method required for moving the robot. Also contains an example implementation (TurtleRobot) that visualizes motion using turtle tools in python.

### main.py
A example file containing usecases for the aforementioned modules.

### config_colors.py
Utility to configure colors. Check log for detailed instructions.

## How to use

Segmentation module installation:
```
cd segment_module
pip3.9 install .
```

Running color configurator:
```
mkdir colors
touch colors/colors.pkl
python3.9 config_colors.py
```

Running an example:
```
python3.9 main.py
```

If you encounter dependency errors, resolve them with pip3.9
