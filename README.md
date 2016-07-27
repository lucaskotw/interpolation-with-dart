# ASF/AMC Parser for DART

This is a project for testing the ASF/AMC parser with motion interpolation using
DART (Dynamic Animation and Robotics Toolkit). The goal is to demonstrate how to
use the ASF/AMC parser.

## Environment

- dart: v6.0
- OS  : Ubuntu 14.04 vivid

## Data

1. [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/)
2. [USC CSCI5250 Computer Animation and Simulation](http://run.usc.edu/cs520-s15/assign2/)


## How to build and execute

1. In the root directory, `mkdir build`.
2. `cd build`
3. `cmake ..`
4. `make MotionInterpolation`
5. `bin/MotionInterpolation \[path to asf data\] \[path to amc data\]`

## Interpolation Method

Linear Interpolation Method with Euler Angle.


## Function of Each File

- AsfParser.h/.cpp: Implement **ASFData** objects to contain data in ASF files.
- AmcParser.h/.cpp: Implement **AMCData** objects to contain data in AMC files,
                    associated with a corresponding ASF data.
- LinearInterpolator.h/.cpp: Implement **LinearInterpolatior** object to 
                             interpolate motion with Euler angle metric.
- MyWindow.h/.cpp: Display the parsed result. Run the interpolation process.
- Main.cpp: The main function. 


## Question

1. How to set the namespaces?
  - file such as the VskParser.h is in the name space 
2. Is there any other suggestion or work to do if there's a change to merge into
   DART project? 

## TODO

- Including Macro parser for the ASF file, such as scaling, axis order, and
  degree unit, etc.
