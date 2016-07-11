#include <algorithm>
#include "AmcParser.h"


//AMCData::AMCData(ASFData asfData)
AMCData::AMCData(ASFData * asfData)
{
  // Set Corresponding ASF Data
  mAsfData = new ASFData();

  std::cout << "create new ASF data pointer" << std::endl;
  *mAsfData = *asfData;

  std::cout << "end recording ASF data" << std::endl;
  // record the nonzero dofs segment name
  std::vector<std::string> asfSegmentNames(0);
  mSegmentNames.resize(0);
  Eigen::VectorXd segmentDofs;
  if (!mAsfData->getSegmentNames(&asfSegmentNames))
  {
    throw /* reading asf segment's name error */;
  }
  for (std::size_t i=0; i<asfSegmentNames.size(); ++i)
  {
    if (asfSegmentNames.at(i) == "root")
    {
      mSegmentNames.push_back("root");
    }
    else if (asfData->getSegmentDegreeOfFreedoms(asfSegmentNames.at(i),
                                                 &segmentDofs))
    {
      if (segmentDofs != Eigen::Vector3d::Zero())
      {
        mSegmentNames.push_back(asfSegmentNames.at(i));
      }
    }
  }

  std::cout << "end loading segment names" << std::endl;

  // set up dofs of the motion sample
  mNumDofs = 0;
  for (std::size_t i=0; i<asfSegmentNames.size(); ++i)
  {
    if (asfSegmentNames.at(i) == "root")
    {
      mNumDofs += 6; // suppose root has 6 dofs
    }
    else if (asfData->getSegmentDegreeOfFreedoms(asfSegmentNames.at(i),
                                                 &segmentDofs))
    {
      if (segmentDofs != Eigen::Vector3d::Zero())
      {
        for (std::size_t j=0; j<segmentDofs.size(); ++j)
        {
          if (segmentDofs(j) == 1) ++mNumDofs;
        }
      }
    }

  }

  std::cout << "end loading segment dofs" << std::endl;

  mNumFrames = 0;
  mFramesConfig.resize(0);
}


AMCData::~AMCData()
{
}


void AMCData::addFrameConfig(SingleFrameConfig frameConfig)
{
  mFramesConfig.push_back(frameConfig);
  ++mNumFrames;
}


// should already know the segment's bonename and dofs pair
bool AMCData::addFrameConfig(Eigen::VectorXd frameConfig)
{
  if (frameConfig.size() != mNumDofs)
  {
    return false;
  }
  else
  {
    SingleFrameConfig addedFrameConfig;
    addedFrameConfig.resize(0);
    std::string segmentName;
    Eigen::VectorXd dofs;
    Eigen::VectorXd dofs_flag;
    int cnt = 0; // record current pointing dof in input flat frame config
    for (std::size_t i=0; i<mSegmentNames.size(); ++i)
    {
      segmentName = mSegmentNames.at(i);
      if (mAsfData->getSegmentDegreeOfFreedoms(segmentName, &dofs_flag))
      {

        dofs.resize(dofs_flag.size());
        dofs.setZero();
        for (std::size_t j=0; j<dofs_flag.size(); ++j)
        {
          if (dofs_flag(j) == 1) // assign dofs with value at this motion frame
          {
            dofs(j) = frameConfig(cnt);
            ++cnt;
          }
        }

        addedFrameConfig.push_back(std::make_pair(segmentName, dofs));
      }
    }
    mFramesConfig.push_back(addedFrameConfig);
    ++mNumFrames;
    return true;
  }
}

int AMCData::getNumDofs()
{
  return mNumDofs;
}


int AMCData::getNumFrames()
{
  return mNumFrames;
}


bool AMCData::getFrameConfig(int frameID, SingleFrameConfig* frameConfig)
{
  if (frameID >= 0 && frameID < mNumFrames)
  {
    frameConfig->resize(0);
    (*frameConfig) = mFramesConfig.at(frameID);
    return true;
  }
  return false;
}



bool AMCData::getSegmentConfig(int frameID,
                               std::string segmentName,
                               Eigen::VectorXd* segmentConfig)
{

  if (frameID >= 0 && frameID < mNumFrames)
  {
    SingleFrameConfig frameConfig = mFramesConfig.at(frameID);
    segmentConfig->resize(0);
    for (int i=0; i<frameConfig.size(); ++i)
    {
      if (frameConfig.at(i).first == segmentName)
      {
        *segmentConfig = frameConfig.at(i).second;
        return true;
      }
    }
    std::cout << "no such segment name in the motion data" << std::endl;
    return false;
  }
  return false;

}


bool AMCData::getFrameConfig(int frameID, Eigen::VectorXd* flatFrameConfig)
{
  
  if (frameID >= 0 && frameID < mNumFrames)
  {
    SingleFrameConfig frameConfig = mFramesConfig.at(frameID);
    flatFrameConfig->resize(mNumDofs);
    int idx = 0;
    for (std::size_t i=0; i<frameConfig.size(); ++i)
    {
      for (std::size_t j=0; j<frameConfig.at(i).second.size(); ++j)
      {
        if (frameConfig.at(i).second(j) != 0)
        {
          (*flatFrameConfig)(idx) = frameConfig.at(i).second(j);
          ++idx;
        }
      }
    }

    return true;
  }

  return false; 
}


std::vector<std::string> AMCData::getSegmentNames()
{
  return mSegmentNames;
}


int AMCData::readAMCFile(char * fileName)
{
  // load file
  std::ifstream is(fileName, std::ios::in); 
  if (is.fail()) return -1;


  // skip macro

  std::istringstream stream;
  std::string line, token;
  while (std::getline(is, line))
  {
    stream.clear();
    stream.str(line);
    stream >> token;
    if (token == ":DEGREES") break;
  }


  // read motion frame
  double dof_val; // dof buffer

  Eigen::VectorXd dofs_flag; // dof indicator for this segment
  Eigen::VectorXd dofs; // positions vector for a segment in a motion frame
  double frameID;
  std::string boneName;
  std::pair<std::string, Eigen::VectorXd> boneConfig;
  SingleFrameConfig frameConfig;
  mFramesConfig.resize(0);
  while(std::getline(is, line))
  {
    stream.clear();
    stream.str(line);
    stream >> frameID;
    if (stream.fail())
    {
      // if not a frame number, then it's a segment data
      dofs.resize(0);
      stream.clear();
      stream.str(line);
      stream >> boneName;
      // if the bone is found
      if (mAsfData->getSegmentDegreeOfFreedoms(boneName, &dofs_flag))
      {
        // make sure dofs size is the same as dof flags
        dofs.resize(dofs_flag.size());
        dofs.setZero();
        for (int i=0; i<dofs_flag.size(); ++i)
        {
          if (dofs_flag(i) == 1) // If the dof flag is 1
          {
            stream >> dof_val;
            dofs(i) = dof_val;
          }
        }

        
        boneConfig = std::make_pair(boneName, dofs);
        frameConfig.push_back(boneConfig);

      }

    }
    else if (frameConfig.size() != 0 && !stream.fail())
    {
      mFramesConfig.push_back(frameConfig);
      ++mNumFrames;
      frameConfig.resize(0);

    }
  }
  // push back last frame's configuration
  mFramesConfig.push_back(frameConfig);
  ++mNumFrames;


  std::cout << "# loaded frames = " << mFramesConfig.size() << std::endl;
  std::cout << "# segments = " << mSegmentNames.size() << std::endl;
      
  is.close();


  return 0;

}

