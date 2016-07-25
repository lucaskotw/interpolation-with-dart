# ASF/AMC Parser for DART

This is a project for testing the ASF/AMC parser with motion interpolation using
DART (Dynamic Animation and Robotics Toolkit).

## Environment

- dart: v6.0
- OS  : Ubuntu 14.04 vivid

## Data

1. [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/)
2. [USC CSCI5250 Computer Animation and Simulation](http://run.usc.edu/cs520-s15/assign2/)

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

1. How to define the namespaces?

## TODO

- Including Macro of the ASF file, such as scaling.
