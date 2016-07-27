#include "MyWindow.h"
#include <unistd.h> // pause for a second


const double deg_to_rad = M_PI/180.0;
const double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter


void MyWindow::setSkel(dart::dynamics::SkeletonPtr _skel)
{
  mSkel = _skel;
}

int MyWindow::loadAndInterpolateMotionData(char* motionFileName,
                                           ASFData * asfData)
{

  // load corresponding ASF data
  mAsfData = new ASFData();
  *mAsfData = *asfData;

  // test motion data
  mInputMotion = new AMCData(asfData);
  mInputMotion->readAMCFile(motionFileName);


  // test lienar interpolation
  mOutputMotion = new AMCData(asfData);
  LinearInterpolator* interpolator = new LinearInterpolator();
  interpolator->linearInterpolation(mInputMotion, mOutputMotion, 12);


  return 0;

}


bool MyWindow::transformSegmentAtSample(std::string segmentName,
                                        int timeStep)
{

  Eigen::VectorXd segmentConfig;
  if (mInputMotion->getSegmentConfig(timeStep, segmentName, &segmentConfig))
  {
    if (segmentName == "root")
    {

      // Transform Root
      Eigen::Vector6d r_t = segmentConfig;

      // r_t:       head(3)->translation, tail(3)->rotation
      // FreeJoint: head(3)->rotation, tail(3)->translation

      Eigen::Isometry3d tf;
      tf.linear() = dart::math::eulerXYZToMatrix(r_t.tail(3)*deg_to_rad);
      //tf.linear() = dart::math::eulerZYXToMatrix(r_t.tail(3)*deg_to_rad);
      tf.translation() = r_t.head(3)*unit;
      mSkel->getJoint("root_joint")
           ->setPositions(dart::dynamics::FreeJoint::convertToPositions(tf));
      return true;
    }
    else
    {
      // other bones
      // prepare reference frame
      Eigen::Vector3d segmentAxes;
      if (mAsfData->getSegmentAxes(segmentName, &segmentAxes))
      {
        Eigen::Matrix3d refFrame = dart::math::eulerXYZToMatrix(segmentAxes*deg_to_rad);
        //Eigen::Matrix3d refFrame = dart::math::eulerZYXToMatrix(segmentAxes*deg_to_rad);
        // calculate motion transformation
        Eigen::Matrix3d motionRot = dart::math::eulerXYZToMatrix(segmentConfig*deg_to_rad);
        //Eigen::Matrix3d motionRot = dart::math::eulerZYXToMatrix(segmentConfig*deg_to_rad);
        Eigen::Isometry3d finalTF;
        //finalTF.linear() = refFrame.inverse() * motionRot * refFrame;
        finalTF.linear() = refFrame * motionRot * refFrame.inverse();
        //finalTF.linear() = motionRot;

        // transform
        mSkel->getBodyNode(segmentName)->getParentBodyNode()->getParentJoint()
             ->setPositions(dart::dynamics::BallJoint::convertToPositions(finalTF.linear()));


        std::cout << segmentName << std::endl;
        std::cout << mSkel->getBodyNode(segmentName)->getParentJoint()
            ->getName() << std::endl;
        std::cout << mSkel->getBodyNode(segmentName)->getRelativeTransform().linear()
                  << std::endl;


        std::cout << mSkel->getJoint(segmentName+"_joint")->getPositions() << std::endl;

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
  if (mTimeStepCnt/12 < mInputMotion->getNumFrames() && mTimeStepCnt % 12 == 0)
  {

    std::vector<std::string> segmentNames = mInputMotion->getSegmentNames();
    //std::vector<std::string> segmentNames = mOutputMotion->getSegmentNames();

    transformSegmentAtSample("root", mTimeStepCnt/12);


    // lower body
    transformSegmentAtSample("lfemur", mTimeStepCnt/12);
    transformSegmentAtSample("ltibia", mTimeStepCnt/12);
    transformSegmentAtSample("lfoot", mTimeStepCnt/12);
    transformSegmentAtSample("ltoes", mTimeStepCnt/12);
    transformSegmentAtSample("rfemur", mTimeStepCnt/12);
    transformSegmentAtSample("rtibia", mTimeStepCnt/12);
    transformSegmentAtSample("rfoot", mTimeStepCnt/12);
    transformSegmentAtSample("rtoes", mTimeStepCnt/12);

     // upper body
    transformSegmentAtSample("lowerback", mTimeStepCnt/12);
    transformSegmentAtSample("upperback", mTimeStepCnt/12);
    transformSegmentAtSample("throax", mTimeStepCnt/12);
    transformSegmentAtSample("lowerneck", mTimeStepCnt/12);
    transformSegmentAtSample("upperneck", mTimeStepCnt/12);
    transformSegmentAtSample("head", mTimeStepCnt/12);

    transformSegmentAtSample("lclavicle", mTimeStepCnt/12);
    transformSegmentAtSample("lhumerus", mTimeStepCnt/12);
    transformSegmentAtSample("lradius", mTimeStepCnt/12);
    transformSegmentAtSample("lwrist", mTimeStepCnt/12);
    transformSegmentAtSample("lhand", mTimeStepCnt/12);
    transformSegmentAtSample("lfingers", mTimeStepCnt/12);
    transformSegmentAtSample("lthumb", mTimeStepCnt/12);
    transformSegmentAtSample("rclavicle", mTimeStepCnt/12);
    transformSegmentAtSample("rhumerus", mTimeStepCnt/12);
    transformSegmentAtSample("rradius", mTimeStepCnt/12);
    transformSegmentAtSample("rwrist", mTimeStepCnt/12);
    transformSegmentAtSample("rhand", mTimeStepCnt/12);
    transformSegmentAtSample("rthumb", mTimeStepCnt/12);

   
    
    //for (std::size_t i=0; i<segmentNames.size(); ++i)
    //{
    //  transformSegmentAtSample(segmentNames.at(i), mTimeStepCnt/12);
    //}

  }

  ++mTimeStepCnt;
  // Step the simulation forward
  mWorld->step();


}
