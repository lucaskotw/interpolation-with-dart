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
                                           ASFData * asfData)
{

  std::cout << "start interpolation process" << std::endl;
  // load corresponding ASF data
  mAsfData = new ASFData();
  *mAsfData = *asfData;

  std::cout << "start interpolation process" << std::endl;
  // test motion data
  mInputMotion = new AMCData(asfData);
  mInputMotion->readAMCFile(motionFileName);

  std::cout << "end loading amc data" << std::endl;

  // test lienar interpolation
  mOutputMotion = new AMCData(asfData);
  LinearInterpolator* interpolator = new LinearInterpolator();
  interpolator->linearInterpolation(mInputMotion, mOutputMotion, 12);

  std::cout << "end interpolation process" << std::endl;

  return 0;

}


bool MyWindow::transformSegmentAtSample(std::string segmentName,
                                        int timeStep)
{

  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter
  Eigen::VectorXd segmentConfig;
  if (mOutputMotion->getSegmentConfig(timeStep, segmentName, &segmentConfig))
  {
    if (segmentName == "root")
    {

      // Transform Root
      Eigen::Vector6d r_t = segmentConfig;
      // Swap head and tail
      Eigen::Vector3d t_tmp = r_t.head(3);
      r_t.head(3) = r_t.tail(3);
      r_t.tail(3) = t_tmp;
      r_t.head(3) *= deg_to_rad;
      r_t.tail(3) *= unit;
      mSkel->getJoint("root_joint")->setPositions(r_t);
      return true;
    }
    else
    {
      // other bones
      // prepare reference frame
      Eigen::Vector3d segmentAxes;
      if (mAsfData->getSegmentAxes(segmentName, &segmentAxes))
      {
        Eigen::Matrix3d refFrame = dart::math::eulerXYZToMatrix(segmentAxes);
        // calculate motion transformation
        Eigen::Matrix3d motionRot = dart::math::eulerXYZToMatrix(segmentConfig);
        //Eigen::Vector3d translationFromParent = mSkel->getJoint(segmentName+"_joint")
        //                                             ->getTransformFromParentBodyNode()
        //                                             .translation();
        Eigen::Isometry3d finalTF;
        finalTF.linear() = refFrame.inverse() * motionRot * refFrame;

        Eigen::Vector3d translationFromParent = mSkel->getBodyNode(segmentName)
          ->getParentJoint()->getTransformFromParentBodyNode().translation();
        finalTF.translation() = translationFromParent;
        //finalTF.linear() = motionRot;

        mSkel->getBodyNode(segmentName)->getParentJoint()
             ->setTransformFromParentBodyNode(finalTF);
        std::cout << segmentName << std::endl;
        std::cout << mSkel->getBodyNode(segmentName)->getParentJoint()
            ->getName() << std::endl;
        //mSkel->getJoint(segmentName+"_joint")
        //     ->setTransformFromParentBodyNode(finalTF);
        return true;

      }
      std::cout << "segment transform fail" << std::endl;
      return false;
    }
  }
  return false;
}


void MyWindow::timeStepping()
{


  //if (mTimeStepCnt/12 < mOutputMotion->getNumFrames() && mTimeStepCnt % 12 == 0)
  if (mTimeStepCnt/120 < mInputMotion->getNumFrames() && mTimeStepCnt % 120 == 0)
  {

    std::vector<std::string> segmentNames = mInputMotion->getSegmentNames();
    //std::vector<std::string> segmentNames = mOutputMotion->getSegmentNames();

    transformSegmentAtSample("root", mTimeStepCnt/120);
    transformSegmentAtSample("lfemur", mTimeStepCnt/120);
    
    //for (std::size_t i=0; i<segmentNames.size(); ++i)
    //{
    //  transformSegmentAtSample(segmentNames.at(i), mTimeStepCnt/12);
    //}

  }

  ++mTimeStepCnt;
  // Step the simulation forward
  mWorld->step();


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
