#include "LinearInterpolator.h"


LinearInterpolator::LinearInterpolator()
{}

LinearInterpolator::~LinearInterpolator()
{}

void LinearInterpolator::linearInterpolation(AMCData * inputMotion,
                                             AMCData * outputMotion,
                                             int windowSize)
{

  std::cout << "start interpolation process" << std::endl;
  std::cout << "input motion size = " << inputMotion->getNumFrames() << std::endl;
  // Planarize the Motion state to a vector
  int numSegments = inputMotion->getSegmentNames().size();
  int numDofs = inputMotion->getNumDofs();

  std::cout << "input motion #dofs = " << numDofs << std::endl;
  Eigen::VectorXd start;
  Eigen::VectorXd end;
  inputMotion->getFrameConfig(0, &start);          // set the first frame as
                                                   // start
  inputMotion->getFrameConfig(windowSize-1, &end); // set the space th frame as
                                                   // end
  std::cout << start << std::endl;
  // interpolate desired frames into the output motion list
  Eigen::VectorXd middle(numDofs);
  double weight;

  int numFrames = inputMotion->getNumFrames();
  int windowCnt = windowSize - 1;
  int space = windowSize - 2;  // remove the start and end frame count
  while (true)
  {
    // add start frame
    if (outputMotion->addFrameConfig(start))
    {
      std::cout << "add start frame" << std::endl;
    }
    else
    {
      std::cout << "fail adding frame" << std::endl;
    }
    // add middle frame
    for (std::size_t i=0; i<space; ++i)
    {
      weight = ((double) i+1/windowSize);

      //std::cout << "weight = " << weight << std::endl;
      middle = (1-weight) * start + weight * end;
    }
    
    // for next iteration
    windowCnt += windowSize;
    start = end;

    // check the terminal process
    if ((numFrames - windowCnt) > windowSize)
    {
      // window size does not exceed the difference between end frame and cnt
      // assign next end frame and continue to next round
      inputMotion->getFrameConfig(windowCnt, &end);
    }
    else
    {
      // put the rest input frames into output frames
      for (std::size_t i=windowCnt; i<numFrames; ++i)
      {
        inputMotion->getFrameConfig(i, &middle);
        outputMotion->addFrameConfig(middle);
      }
      break;
    }

  }

  std::cout << "output motion size = " << outputMotion->getNumFrames() << std::endl;


}

