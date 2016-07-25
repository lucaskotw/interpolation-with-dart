#ifndef MOTIONINTERPOLATION_ASFPARSER_H_
#define MOTIONINTERPOLATION_ASFPARSER_H_

#include <dart/dart.hpp>
#include <vector>
#include <utility> // pair
#include <regex> // extract pair from limit
#include <string>
#include <fstream>
#include <sstream>



const int DOF_NONE     = 0;
const int DOF_RX       = 1;
const int DOF_RY       = 2;
const int DOF_RZ       = 4;
const int DOF_RX_RY    = 3;
const int DOF_RX_RZ    = 5;
const int DOF_RY_RZ    = 6;
const int DOF_RX_RY_RZ = 7;


struct Root
{
  //char * order;
  //char * axis;
  std::vector<double> position;
  std::vector<double> orientation;
};


struct Bone
{
  int id;
  std::string name;
  Eigen::Vector3d direction;
  double length;
  int dof_flag;
  Eigen::Vector3d axes;
  Eigen::Vector3d dofs;
  std::vector<std::pair<double, double>> limits;
};


class ASFData
{
private:
  // members
  std::ifstream mRetriever;
  Root mRoot;
  std::vector<Bone> mBones;

  // getter for Root and Bones, which are supposed to be accessed only by
  // instance itself
  bool getBone(Bone * target, std::string boneName);

  // subroutines for reading the data from ASF
  bool setBones();
  bool setRoot();
  bool generateSkeletonHierarchy(dart::dynamics::SkeletonPtr skel);


public:
  // constructor, copy constructor and destructor
  ASFData();
  ASFData(const ASFData & asfData);
  ASFData& operator= (const ASFData & asfData);
  ~ASFData();

  // getter and setter
  bool getSegmentDirection(std::string segmentName,
                           Eigen::Vector3d * direction);
  bool getSegmentLength(std::string segmentName,
                        double * length);
  bool getSegmentAxes(std::string segmentName,
                      Eigen::Vector3d * segmentAxes);
  bool getSegmentDegreeOfFreedoms(std::string segmentName,
                                  Eigen::VectorXd * segmentDofs);
  bool getSegmentLimits(std::string segmentName,
                        std::vector<std::pair<double, double>>* limits);
  bool getSegmentNames(std::vector<std::string> * segmentNameList);

  // load data from reading the ASF file
  bool readSkeleton(char * fileName, dart::dynamics::SkeletonPtr skel);

};


#endif
