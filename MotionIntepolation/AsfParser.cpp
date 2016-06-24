/*
 * DART Version: 5.1
 *
 */
#include "AsfParser.h"
#include <iostream> // demonstrate the result


const double joint_radius = 0.02; // m
const double bone_radius = 0.02; // m


using namespace dart::dynamics;
using namespace dart::simulation;



ASFBoneData::ASFBoneData()
{

}


ASFBoneData::~ASFBoneData()
{}


void ASFBoneData::setRoot()
{
  std::string line, token, dof_val;
  while (std::getline(retriever, line))
  {
    std::istringstream stream(line);
    stream >> token;

    std::cout << token << std::endl;
    if (token == ":bonedata")
      break;

    // read the order
    if (token == "order")
    {
      //stream >> root->order;
    }

    // read the axis order
    if (token == "axis")
    {
      //stream >> root->axis;
    }

    // read the direction
    if (token == "direction")
    {
      //stream >> root->direction;
    }

    // read the orientation
    if (token == "orientation")
    {
      //stream >> root->orientation;
    }


  }

  std::cout << "finish setting root" << std::endl;
}


void ASFBoneData::readBones()
{
  std::string line, token, dof_val;
  bool is_seg_begin = false;
  Bone newBone;
  std::istringstream stream;
  std::string dof_flag; // dof flag
  double direction_buff; // direction buffer
  /*
  std::string limit_buff; // limit buffer
  std::smatch limit_match;
  std::regex  limit_pattern(R"(([+-]{0,1}[0-9]+\.[0-9]+))");
  */
  //std::regex  limit_pattern("-?[0-9]+(\\.[0-9]+)?$");
  std::pair<double, double> limits; // limit pair temp

  while (std::getline(retriever, line))
  {
    stream.clear();
    stream.str(line);
    stream >> token;

    if (token == ":hierarchy")
    {
      std::cout << "end of bone data" << std::endl;
      break;
    } 
    // read begin -> start new segment
    if (token == "begin")
    {
      is_seg_begin = true;
      newBone.dof = DOF_NONE; // make sure the segment starts with 0 DOF
      newBone.direction = Eigen::Vector3d::Zero(); // start new bone's direction
      newBone.limits.resize(0); // start new bone's limits pair
    }

    
    while (is_seg_begin)
    {
      // input new string to stream
      stream.clear();
      std::getline(retriever, line);
      stream.str(line);
      
      stream >> token;

      // read end -> end old segment
      if (token == "end")
      {


        std::cout << "segment ended" << std::endl;
        is_seg_begin = false;
        // push the new bone to bone list
        bones.push_back(newBone);
        // print current data
        break;
      }

      // read id
      if (token == "id")
      {

        stream >> newBone.id;

      }


      // read name -> replace current bone_name to read one
      if (token == "name")
      {
        stream >> newBone.name;

      }


      // read direction
      if (token == "direction")
      {
        for (int i=0; i<3; ++i) // suppose direction is 3D
        {
          stream >> direction_buff;
          newBone.direction(i) = direction_buff;
        }

      }


      // read length
      if (token == "length")
      {

        stream >> newBone.length;
      }


      // read degree of freedom
      if (token == "dof")
      {
        while(stream >> dof_flag)
        {
          if (dof_flag == "rx")
            newBone.dof += DOF_RX;
          if (dof_flag == "ry")
            newBone.dof += DOF_RY;
          if (dof_flag == "rz")
            newBone.dof += DOF_RZ;
        }
      }


      // read dof limitation
      if (token == "limits")
      {
        /*
        // first limits pair
        stream >> limit_buff;
        std::regex_match(limit_buff, limit_match, limit_pattern);
        limits.first = std::stod(limit_match[1]);

        stream >> limit_buff;
        std::regex_match(limit_buff, limit_match, limit_pattern);
        limits.second = std::stod(limit_match[1]);
        newBone.limits.push_back(limits);
*/
      }


    } // end of reading bone segment

  } // end of reading bone data

  std::cout << "finish reading Bones" << std::endl;
  std::cout << "Bones" << std::endl;
  for (std::vector<Bone>::iterator it=bones.begin(); it!=bones.end(); ++it)
  {
    std::cout << "id = " << std::to_string((*it).id)
              << ", name = " << (*it).name
              << ", dof = " << (*it).dof
              << ", length = " << (*it).length;
    std::cout << ", direction = " << (*it).direction;

    std::cout << std::endl;
  }

}


BodyNodePtr createRoot(
    const SkeletonPtr& skel,
    BodyNodePtr parent,
    Root root)
{
  // define the unit conversion
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter

  BodyNodePtr bn;

  FreeJoint::Properties j_prop;
  j_prop.mName = "root_joint";
  BodyNode::Properties b_prop;
  b_prop.mName = "root";
  parent = nullptr;

  bn = skel->createJointAndBodyNodePair<FreeJoint>(
    parent, j_prop, b_prop).second;

  // make joint shape
  const double& j_R = joint_radius;
  std::shared_ptr<EllipsoidShape> j_shape(
      new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_R, j_R, j_R)));
  j_shape->setColor(dart::Color::Blue());
    bn->addVisualizationShape(j_shape);



  return bn; 
}



BodyNodePtr createSegment(
    const SkeletonPtr& skel,
    BodyNodePtr parent,
    Bone& bone)
{
  // define the unit conversion
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter

  BodyNodePtr bn;

  BallJoint::Properties j_prop;
  j_prop.mName = bone.name + "_joint";
  j_prop.mT_ParentBodyToJoint.translation() = bone.length * unit * bone.direction;
  BodyNode::Properties b_prop;
  b_prop.mName = bone.name;
  bn = skel->createJointAndBodyNodePair<BallJoint>(
    parent, j_prop, b_prop).second;

  // make joint shape
  const double& j_R = joint_radius;
  std::shared_ptr<EllipsoidShape> j_shape(
      new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_R, j_R, j_R)));
  j_shape->setColor(dart::Color::Red());
  bn->addVisualizationShape(j_shape);

  // make lhipjoint body shape
  // Create a CylinderShape to be used for both visualization and
  // collision checking
  std::shared_ptr<CylinderShape> b_shape(
      new CylinderShape(j_R, bone.length * unit));
  b_shape->setColor(dart::Color::Black());

  Joint* p_j = bn->getParentJoint();
  Eigen::Isometry3d tf = p_j->getTransformFromParentBodyNode();
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();

  // TODO: This could be substitute by computeRotation in 6.0
  Eigen::Matrix3d rot_m;
  Eigen::Vector3d axis0 = tf.translation().normalized();
  Eigen::Vector3d axis1 = axis0.cross(Eigen::Vector3d::UnitX());
  axis1.normalize();
  Eigen::Vector3d axis2 = axis0.cross(axis1).normalized();
  int index = 2; // start from z
  rot_m.col(index)       = axis0;
  rot_m.col(++index % 3) = axis1;
  rot_m.col(++index % 3) = axis2;
  localTransform.linear() = rot_m;


  localTransform.translation() = 0.5 * tf.translation();
  b_shape->setLocalTransform(localTransform);
  // Add it as a visualization and collision shape
  bn->getParentBodyNode()->addVisualizationShape(b_shape);


  return bn; 
}


void ASFBoneData::generateSkeletonHierarchy(dart::dynamics::SkeletonPtr& skel)
{

  // create root
  BodyNodePtr parentBodyNode, childBodyNode;
  parentBodyNode = createRoot(skel, nullptr, getRoot());

  // attach bones

  std::istringstream stream;
  std::string line, parentBoneName, childBoneName;
  Bone childBone;
  while (std::getline(retriever, line))
  {
    stream.clear();
    stream.str(line);
    stream >> parentBoneName;
    parentBodyNode = skel->getBodyNode(parentBoneName);
    while(stream >> childBoneName)
    {
      if (getBone(childBone, childBoneName))
        childBodyNode = createSegment(skel, parentBodyNode, childBone);
    }
  }


  std::cout << "finish generating root" << std::endl;
}


bool ASFBoneData::getBone(Bone& target, std::string boneName)
{
  for (std::vector<Bone>::iterator it=bones.begin(); it!=bones.end(); ++it)
  {
    if ((*it).name == boneName)
    {
      target = *it;
      return true;
    }
  }
  return false;
}


Root ASFBoneData::getRoot()
{
  return root;
}


dart::dynamics::SkeletonPtr ASFBoneData::readSkeleton(char * fileName)
{

  // init retriever
  retriever.open(fileName, std::ios::in);
  assert(retriever);

  // skip macro
  std::string line, token, dof_val;
  while (std::getline(retriever, line))
  {
    std::istringstream stream(line);
    stream >> token;
    if (token == ":root") break;
  }


  // read root data
  setRoot();


  // add Bone data
  readBones();


  // generate the whole skeleton with hierarchy structure
  dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create("human");
  generateSkeletonHierarchy(skel);


  return skel;
}
