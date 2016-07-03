#include <algorithm>
#include "AmcParser.h"


//AMCData::AMCData(ASFData asfData)
AMCData::AMCData()
{
  // Set Corresponding ASF Data
  //mAsfData = asfData;

  mNumFrames = 0;
  mNumDofs   = 0;
  mFramesConfig.resize(0);
  mAsfSegmentData.resize(0);
}


AMCData::~AMCData()
{
}


// TODO: this should be added to constructor
void AMCData::setNumDofs(
    std::vector<std::pair<std::string, double>> segmentNameAndDofs
)
{
  int dofs = 0;
  for (int i=0; i<segmentNameAndDofs.size(); ++i)
    dofs += segmentNameAndDofs.at(i).second;
  mNumDofs = dofs;
}




void AMCData::addFrameConfig(SingleFrameConfig frameConfig)
{
  mFramesConfig.push_back(frameConfig);
  ++mNumFrames;
}


// should already know the segment's bonename and dofs pair
bool AMCData::addFrameConfig(
    Eigen::VectorXd frameConfig,
    std::vector<std::pair<std::string, double>> segmentNameAndDofs
)
{
  if (frameConfig.size() != mNumDofs)
    return false;
  else
  {
    SingleFrameConfig addedFrameConfig;
    addedFrameConfig.resize(0);
    std::string segmentName;
    std::vector<double> dofs;
    int cnt = 0; // record current pointing dof
    for (int i=0; i<segmentNameAndDofs.size(); ++i)
    {
      segmentName = segmentNameAndDofs.at(i).first;
      dofs.resize(0);
      for (int j=0; j<segmentNameAndDofs.at(i).second; ++j)
      {
        dofs.push_back(frameConfig(cnt));
        ++cnt;
      }
      addedFrameConfig.push_back(std::make_pair(segmentName, dofs));
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
    if (mAsfData.getSegmentConfig
    std::vector<double> dofs(0);
    for (int i=0; i<frameConfig.size(); ++i)
    {
      if (frameConfig.at(i).first == segmentName)
      {
        dofs = frameConfig.at(i).second;
      }
    }
    if (dofs.size() != 0)
    {
      return true;
    }
    std::cout << "no such segment name in the motion data" << std::endl;
    *frameConfig = mFramesConfig.at(frameID);
    return ;
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
    for (int i=0; i<frameConfig.size(); ++i)
    {
      for (int j=0; j<frameConfig.at(i).second.size(); ++j)
      {
        (*flatFrameConfig)(idx) = frameConfig.at(i).second.at(j);
        ++idx;
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
  std::vector<double> dofs; // whole vector of dofs for a bone
  std::pair<std::string, std::vector<double>> boneConfig;
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
      if (mAsfData.getSegmentDegreeOfFreedoms(boneName, &dofs_flag))
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

        // add segment name at the first round
        if (mNumFrames == 0) mSegmentNames.push_back(boneName);
      }

    }
    else if (frameConfig.size() != 0 && !stream.fail())
    {
      // if this is the first sample, push the segment name into the
      // mSegmentNames
      if (mNumFrames == 0)
      {
        mSegmentNames.resize(0);
        for (int i=0; i<frameConfig.size(); ++i)
        {
          mSegmentNames.push_back(frameConfig.at(i).first);
        }
      }
 
      mFramesConfig.push_back(frameConfig);
      ++mNumFrames;
      frameConfig.resize(0);

    }
  }
  // push back last frame's configuration
  mFramesConfig.push_back(frameConfig);
  ++mNumFrames;


  // set the number of degree of freedom
  for (int i=0; i<frameConfig.size(); ++i)
  {
    for (int j=0; j<frameConfig.at(i).second.size(); ++j)
      ++mNumDofs;
  }

  std::cout << "# loaded frames = " << mFramesConfig.size() << std::endl;
  std::cout << "# segments = " << mSegmentNames.size() << std::endl;
      
  is.close();


  return 0;

}

