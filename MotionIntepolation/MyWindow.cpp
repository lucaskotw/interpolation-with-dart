#include "MyWindow.h"
#include <unistd.h> // pause for a second


const double deg_to_rad = M_PI/180.0;
const int paused_mili_seconds = 8;


MyWindow::MyWindow(dart::simulation::WorldPtr world)
//    : SimWindow()
{
  mTimeStepCnt = 0;
  std::cout << "world time step" << std::endl;
  std::cout << world->getTimeStep() << std::endl;
  setWorld(world);


  mSkel = world->getSkeleton("robot");
  // mZooming = false;

  // make sure the world is not intefered with gravity
  Eigen::Vector3d zero_g = Eigen::Vector3d::Zero();
  mWorld->setGravity(zero_g);


}


MyWindow::~MyWindow() {
}


int MyWindow::loadAndInterpolateMotionData(char* motionFileName,
                                           ASFData asfData)
{
  // test motion data
  //mInputMotion = new AMCData(asfData);
  mInputMotion = new AMCData();
  //mInputMotion->readAMCFile(motionFileName);

  // test lienar interpolation
  //mOutputMotion = new AMCData(asfData);
  mOutputMotion = new AMCData();
  //LinearInterpolator* interpolator = new LinearInterpolator();
  //interpolator->linearInterpolation(mInputMotion, mOutputMotion, 12, 12);


  return 0;

}


void MyWindow::timeStepping()
{

  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter

  // Transform Root
#if 0
  if (mTimeStepCnt/12 < mOutputMotion->getNumFrames() && mTimeStepCnt % 12 == 0)
  {
    std::cout << "Transform Root" << std::endl;
    SingleFrameConfig frameConfig;
    mOutputMotion->getFrameConfig(mTimeStepCnt/12, &frameConfig);
    Eigen::Vector6d r_t(frameConfig.at(0).second.data());
    // Swap head and tail
    Eigen::Vector3d t_tmp = r_t.head(3);
    r_t.head(3) = r_t.tail(3);
    r_t.tail(3) = t_tmp;
    r_t.head(3) *= deg_to_rad;
    r_t.tail(3) *= unit;
    mSkel->getJoint("root_joint")->setPositions(r_t);
  }
#else
  if (mTimeStepCnt/12 < mInputMotion->getNumFrames() && mTimeStepCnt % 12 == 0)
  {
    //std::cout << "Transform Root" << std::endl;
    SingleFrameConfig frameConfig;
    mInputMotion->getFrameConfig(mTimeStepCnt/12, &frameConfig);
    Eigen::Vector6d r_t(frameConfig.at(0).second.data());
    // Swap head and tail
    Eigen::Vector3d t_tmp = r_t.head(3);
    r_t.head(3) = r_t.tail(3);
    r_t.tail(3) = t_tmp;
    r_t.head(3) *= deg_to_rad;
    r_t.tail(3) *= unit;
    mSkel->getJoint("root_joint")->setPositions(r_t);
/*
    // Transform each bone
    mSkel->getBodyNode("lowerback")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lowerback"));

    //std::cout << "lowerback change" << std::endl;

    mSkel->getBodyNode("upperback")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "upperback"));

    //std::cout << "upperback change" << std::endl;

    mSkel->getBodyNode("thorax")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "thorax"));

    //std::cout << "thorax change" << std::endl;

    mSkel->getBodyNode("lowerneck")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lowerneck"));

  //std::cout << "lowerneck change" << std::endl;
    mSkel->getBodyNode("upperneck")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "upperneck"));

  //std::cout << "upperneck change" << std::endl;

  mSkel->getBodyNode("head")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "head"));
  //std::cout << "head change" << std::endl;

  mSkel->getBodyNode("rclavicle")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rclavicle"));

  //std::cout << "rclavicle change" << std::endl;


  mSkel->getBodyNode("rhumerus")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rhumerus"));


  //std::cout << "rhumerus change" << std::endl;
  mSkel->getBodyNode("rradius")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rradius"));

  //std::cout << "rradius change" << std::endl;

  mSkel->getBodyNode("rwrist")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rwrist"));

  //std::cout << "rwrist change" << std::endl;

  mSkel->getBodyNode("rhand")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rhand"));

  //std::cout << "rhand change" << std::endl;

  mSkel->getBodyNode("rfingers")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rfingers"));


  //std::cout << "rfingers change" << std::endl;
  mSkel->getBodyNode("rthumb")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "rthumb"));


  //std::cout << "rthumb change" << std::endl;
  mSkel->getBodyNode("lclavicle")->getParentJoint()->setPositions(
    deg_to_rad*mo.getBoneTransform(mMotionCnt, "lclavicle"));

  //std::cout << "lclavicle change" << std::endl;

  mSkel->getBodyNode("lhumerus")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lhumerus"));

  //std::cout << "lhumerus change" << std::endl;
  mSkel->getBodyNode("lradius")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lradius"));

  //std::cout << "lradius change" << std::endl;

  mSkel->getBodyNode("lwrist")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lwrist"));

  //std::cout << "lwrist change" << std::endl;

  mSkel->getBodyNode("lhand")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lhand"));


  //std::cout << "lhand change" << std::endl;
    mSkel->getBodyNode("lfingers")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lfingers"));


  //std::cout << "lfingers change" << std::endl;
    mSkel->getBodyNode("lthumb")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lthumb"));

  //std::cout << "lthumb change" << std::endl;
  mSkel->getBodyNode("rfemur")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "rfemur"));
*/

  }

#endif
  ++mTimeStepCnt;
  // Step the simulation forward
  mWorld->step();

/*
    //std::cout << "rfemur change" << std::endl;
  mSkel->getBodyNode("rtibia")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "rtibia"));

  //std::cout << "rtibia change" << std::endl;

  mSkel->getBodyNode("rfoot")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "rfoot"));

  //std::cout << "rfoot change" << std::endl;

  mSkel->getBodyNode("rtoes")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "rtoes"));


  //std::cout << "rtoes change" << std::endl;
  mSkel->getBodyNode("lfemur")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lfemur"));

  //std::cout << "lfemur change" << std::endl;
  mSkel->getBodyNode("ltibia")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "ltibia"));


  //std::cout << "ltibia change" << std::endl;
  mSkel->getBodyNode("lfoot")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "lfoot"));

  //std::cout << "lfoot change" << std::endl;

  mSkel->getBodyNode("ltoes")->getParentJoint()->setPositions(
      deg_to_rad*mo.getBoneTransform(mMotionCnt, "ltoes"));

  ++mMotionCnt;
  std::cout << mSkel->getState() << std::endl;
  //std::cout << "ltoes change" << std::endl;

  // Move to next counter (skip 3 frame

  //if (mMotionCnt < 4840) mMotionCnt += 3;
  //else getchar();
  }
  ++stepCnt;

  std::cout << "current frame id = " << mMotionCnt << std::endl;
  std::cout << "Gravity" << mWorld->getGravity() << std::endl;  
*/
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  switch (_key)
  {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
      {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
      {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating)
      {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating)
      {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}
