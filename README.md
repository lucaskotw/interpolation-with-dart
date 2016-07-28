# ASF/AMC Parser for DART

This is a project for testing the ASF/AMC parser with motion interpolation using
[DART (Dynamic Animation and Robotics Toolkit)](http://dartsim.github.io). The goal is to demonstrate how to
use the ASF/AMC parser.

## Format Discription

ASF file is for skeleton building.
AMC file is for motion data which could be implemented on the built skeleton.

## Environment

- DART         : v6.0
- OS           : Ubuntu 14.04 vivid
- Cmake Version: 2.8

## Data

[CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/)

### ASF/AMC pairing

| ASF     | AMCs                 | Motion Discription |
|---------|----------------------|--------------------|
| 02.asf  | 02_02.amc            | walk               |
| 07.asf  | 07_05.amc            | walk               |
| 09.asf  | 09_06.amc            | run                |
| 13.asf  | 13_17.amc, 13_18.amc | boxing             |
| 131.asf | 131_04.amc           | dance              |
| 135.asf | 135_06.amc           | martial arts       |

## How to Build and Execute

1. In the root directory, `mkdir build`.
2. `cd build`
3. `cmake ..`
4. `make MotionInterpolation`
5. `bin/MotionInterpolation (path to asf data) (path to amc data)`

## The Interpolation Method

Linear Interpolation Method with Euler Angle.


## Function of Each File

- `AsfParser.h/.cpp`: Implement **ASFData** objects to contain data in ASF files.
- `AmcParser.h/.cpp`: Implement **AMCData** objects to contain data in AMC files,
                    associated with a corresponding ASF data.
- `LinearInterpolator.h/.cpp`: Implement **LinearInterpolatior** object to 
                             interpolate motion with Euler angle metric.
- `MyWindow.h/.cpp`: Display the parsed result. Run the interpolation process.
- `Main.cpp`: The main function.


## The Concept of Skeleton Building and Motion Processing

### Skeleton Building
The method I use is `createJointAndBodyNodePair`.

The following figure explain the idea.

### Motion Processing

Configuration of the skeleton will reset by the following processes.


## Questions

1. How to set the namespaces?
  - file such as the VskParser.h is in the name space `dart::utils` and code
    like follows

        dart {
            util {
        
                code block
            }
        }

2. Is there any other suggestion or work to do if there's a chance to merge into
   the major DART project? 

3. Developing Questions: Would you use std::cout to indicate which functions is
   executing now? Or you just use gdb to develop and debug?

## TODOs

- Including macro parser for the *ASF* file, such as *scaling*, *axis order*,
  and *degree unit*, etc.
- Verifying some degree in *AMC* file over 360 deg
- Including limit parser for each segment
