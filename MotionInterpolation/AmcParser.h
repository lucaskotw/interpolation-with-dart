#ifndef DISPLAYMOTION_AMCPARSER_H_
#define DISPLAYMOTION_AMCPARSER_H_


#include <vector>
#include <utility>
#include <fstream>
#include <string>
#include "dart/dart.h"
#include "AsfParser.h"

// Motion Data Type Definition
#define SingleFrameConfig std::vector<std::pair<std::string, Eigen::VectorXd>>


struct AMCSegmentData
{
  std::string name;
  Eigen::VectorXd dofs;
};


class AMCData
{
public:
  AMCData(ASFData * asfData);
  ~AMCData();
  int readAMCFile(char * fileName);

  // Often used in explicitly insert reference frame
  void addFrameConfig(SingleFrameConfig frameConfig);

  // Added in the case that the single frame config is planarized vector.
  bool addFrameConfig(Eigen::VectorXd frameConfig);
  bool getFrameConfig(int frameID, SingleFrameConfig* frameConfig);
  bool getFrameConfig(int frameID, Eigen::VectorXd* flatFrameConfig);
  bool getSegmentConfig(int frameID,
                        std::string segmentName,
                        Eigen::VectorXd* segmentConfig);
                                 
  std::vector<std::string> getSegmentNames();
  int getNumDofs();
  int getNumFrames();

private:
  std::vector<SingleFrameConfig> mFramesConfig;
  std::vector<std::string> mSegmentNames;
  ASFData * mAsfData;
  int mNumFrames;
  int mNumDofs;

};


#endif
