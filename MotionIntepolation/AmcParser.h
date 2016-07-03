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
  //AMCData(ASFData asfData);
  AMCData();
  ~AMCData();
  int readAMCFile(char * fileName);
  void setNumDofs(std::vector<std::pair<std::string, double>> segmentNameAndDofs);

  void addFrameConfig(SingleFrameConfig frameConfig);
  bool addFrameConfig(Eigen::VectorXd frameConfig,
    std::vector<std::pair<std::string, double>> segmentNameAndDofs
  );
  bool getFrameConfig(int frameID, SingleFrameConfig* frameConfig);
  bool getFrameConfig(int frameID, Eigen::VectorXd* flatFrameConfig);
  bool getSegmentConfig(int frameID,
                        std::string segmentName,
                        Eigen::VectorXd* segmentConfig);
                                 
  //std::vector<std::string> getSegmentNames();
  int getNumDofs();
  int getNumFrames();

private:
  std::vector<SingleFrameConfig> mFramesConfig;
  std::vector<AMCSegmentData> mSegmentData;
  std::vector<std::string> mSegmentNames;
  ASFData * mAsfData;
  int mNumFrames;
  int mNumDofs;

};


#endif
