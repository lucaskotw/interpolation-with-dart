/**
 * Author: Lucas Ko <lucaskointw@gmail.com>
 *
 * National Taiwan University
 */
#ifndef DISPLAYMOTION_MYWINDOW_H_
#define DISPLAYMOTION_MYWINDOW_H_

#include "dart/dart.h"
#include <iostream> // use to display the reading result
#include <fstream>  // load data
#include <string>   // for line buffer
#include "AmcParser.h"
#include "LinearInterpolator.h"

class MyWindow : public dart::gui::SimWindow {
public:
  // Constructor
  MyWindow(dart::simulation::WorldPtr world);

  ~MyWindow();


  int loadAndInterpolateMotionData(char* motionFileName,
                                   ASFData * asfData);
  bool transformSegmentAtSample(std::string segmentName, int timeStep);
  void timeStepping() override;
  void keyboard(unsigned char _key, int _x, int _y) override;

private:
  ASFData * mAsfData;
  AMCData* mInputMotion;
  AMCData* mOutputMotion;
  int mTimeStepCnt;
  dart::dynamics::SkeletonPtr mSkel;


};


#endif
