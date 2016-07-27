# ASF/AMC Parser for DART

This is a project for testing the ASF/AMC parser with motion interpolation using
DART (Dynamic Animation and Robotics Toolkit). The goal is to demonstrate how to
use the ASF/AMC parser.

## Format Discription



## Environment

- DART         : v6.0
- OS           : Ubuntu 14.04 vivid
- Cmake Version: 2.8

## Data

1. [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/)
2. [USC CSCI5250 Computer Animation and Simulation Assignment 2](http://run.usc.edu/cs520-s15/assign2/)

### ASF/AMC pairing

- `02.asf` with `02_02.amc`
- `13.asf` with `13_17.amc` and `13_18.amc`

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

## TODOs

- Including macro parser for the *ASF* file, such as *scaling*, *axis order*,
  and *degree unit*, etc.
- Verifying some degree in *AMC* file over 360 deg
