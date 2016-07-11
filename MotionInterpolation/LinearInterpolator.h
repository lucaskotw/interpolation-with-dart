/**
 * SPEC of motion interpolation
 * Approach: Linear Interpolation
 * Keyframe selection: uniformly pick
 * Based on a list of LinearPathSegment
 */
#ifndef _MOTIONINTERPOLATION_LINEARINTERPOLATOR_H
#define _MOTIONINTERPOLATION_LINEARINTERPOLATOR_H

#include "AmcParser.h"
#include <dart/dart.h>




class LinearInterpolator
{
  public:
    LinearInterpolator();
    ~LinearInterpolator();
    Eigen::VectorXd getConfig(int time);
    void linearInterpolation(AMCData* inputMotion,
                             AMCData* outputMotion,
                             int windowSize);

};
#endif
