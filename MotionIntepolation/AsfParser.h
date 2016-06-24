#ifndef DISPLAYMOTION_ASFPARSER_H_
#define DISPLAYMOTION_ASFPARSER_H_

#include "dart/dart.h"
#include <vector>
#include <utility> // pair
#include <regex> // extract pair from limit
#include <string>
#include <fstream>
#include <sstream>



#define DOF_NONE     0
#define DOF_RX       1
#define DOF_RY       2
#define DOF_RZ       4
#define DOF_RX_RY    3
#define DOF_RX_RZ    5
#define DOF_RY_RZ    6
#define DOF_RX_RY_RZ 7


struct Root
{
  char * order;
  char * axis;
  std::vector<double> position;
  std::vector<double> orientation;
};

struct Bone
{
  int id;
  std::string name;
  Eigen::Vector3d direction;
  double length;
  int dof;
  std::vector<std::pair<double, double>> limits;
};


class ASFBoneData
{
private:
  std::ifstream retriever;
  Root root;
  std::vector<Bone> bones;

public:
  ASFBoneData();
  ~ASFBoneData();
  void readBones();
  void setRoot();
  void generateSkeletonHierarchy(dart::dynamics::SkeletonPtr& skel);

  bool getBone(Bone& target, std::string boneName);
  Root getRoot();


  dart::dynamics::SkeletonPtr readSkeleton(char * fileName);
};


#endif
