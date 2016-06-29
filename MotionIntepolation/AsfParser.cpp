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


/* Constructor and Destructor */
ASFData::ASFData()
{

}


ASFData::~ASFData()
{}


/* getter for a specific Bone, which are supposed to be accessed only by
 * instance itself
 */
bool ASFData::getBone(Bone * target, std::string boneName)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == boneName)
    {
      *target = mBones.at(i);
      return true;
    }
  }
  return false;
}


// getter for segment
bool ASFData::getSegmentDirection(std::string segmentName,
                           Eigen::Vector3d * direction)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == segmentName)
    {
      *direction = mBones.at(i).direction;
      return true;
    }
  }
  return false;

}


bool ASFData::getSegmentLength(std::string segmentName, double * length)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == segmentName)
    {
      *length = mBones.at(i).length;
      return true;
    }
  }
  return false;

}


bool ASFData::getSegmentDegreeOfFreedoms(std::string segmentName,
                                         Eigen::Vector3d * segmentDofs)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == segmentName)
    {
      *segmentDofs = mBones.at(i).dofs;
      return true;
    }
  }
  return false;

}


bool ASFData::getSegmentLimits(std::string segmentName,
                               std::vector<std::pair<double, double>>* limits)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == segmentName)
    {
      *limits = mBones.at(i).limits;
      return true;
    }
  }
  return false;

}


/* subroutines for reading the data from ASF */
bool ASFData::setRoot()
{
  std::string line, token, dof_val;
  while (std::getline(mRetriever, line))
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
  return true;
}


bool ASFData::setBones()
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

  while (std::getline(mRetriever, line))
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
      newBone.dof_flag = DOF_NONE; // make sure the segment starts with 0 DOF
      newBone.direction = Eigen::Vector3d::Zero(); // start new bone's direction
      newBone.limits.resize(0); // start new bone's limits pair
    }

    
    while (is_seg_begin)
    {
      // input new string to stream
      stream.clear();
      std::getline(mRetriever, line);
      stream.str(line);
      
      stream >> token;

      // read end -> end old segment
      if (token == "end")
      {


        std::cout << "segment ended" << std::endl;
        is_seg_begin = false;
        // push the new bone to bone list
        mBones.push_back(newBone);
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
            newBone.dof_flag += DOF_RX;
          if (dof_flag == "ry")
            newBone.dof_flag += DOF_RY;
          if (dof_flag == "rz")
            newBone.dof_flag += DOF_RZ;
        }

        if (newBone.dof_flag == DOF_NONE)
        {
          newBone.dofs = Eigen::Vector3d(0, 0, 0);
        }
        else if (newBone.dof_flag == DOF_RX)
        {
          newBone.dofs = Eigen::Vector3d(1, 0, 0);
        }
        else if (newBone.dof_flag == DOF_RY)
        {
          newBone.dofs = Eigen::Vector3d(0, 1, 0);
        }
        else if (newBone.dof_flag == DOF_RZ)
        {
          newBone.dofs = Eigen::Vector3d(0, 0, 1);
        }
        else if (newBone.dof_flag == DOF_RX_RY)
        {
          newBone.dofs = Eigen::Vector3d(1, 1, 0);
        }
        else if (newBone.dof_flag == DOF_RX_RZ)
        {
          newBone.dofs = Eigen::Vector3d(1, 0, 1);
        }
        else if (newBone.dof_flag == DOF_RY_RZ)
        {
          newBone.dofs = Eigen::Vector3d(0, 1, 1);
        }
        else if (newBone.dof_flag == DOF_RX_RY_RZ)
        {
          newBone.dofs = Eigen::Vector3d(1, 1, 1);
        }

      } // end reading degree of freedom


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
  for (std::vector<Bone>::iterator it=mBones.begin(); it!=mBones.end(); ++it)
  {
    std::cout << "id = " << std::to_string((*it).id)
              << ", name = " << (*it).name
              << ", dof_flag = " << (*it).dof_flag
              << ", length = " << (*it).length;
    std::cout << ", direction = " << (*it).direction;

    std::cout << std::endl;
  }
  return true;

}


// Subroutine for hierarchical generate skeleton
BodyNodePtr createRoot(
    SkeletonPtr skel,
    BodyNodePtr parent,
    Root * root)
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
    SkeletonPtr skel,
    BodyNodePtr parent,
    Bone * bone)
{
  // define the unit conversion
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter

  BodyNodePtr bn;

  std::cout << "bone name = " << bone->name << std::endl;

  BallJoint::Properties j_prop;
  j_prop.mName = bone->name + "_joint";
  j_prop.mT_ParentBodyToJoint.translation() = bone->length * unit * bone->direction;
  BodyNode::Properties b_prop;
  b_prop.mName = bone->name;
  bn = skel->createJointAndBodyNodePair<BallJoint>(
    parent, j_prop, b_prop).second;

  std::cout << "bone name = " << bn->getName() << std::endl;
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
      new CylinderShape(j_R, bone->length * unit));
  b_shape->setColor(dart::Color::Black());

  Joint* p_j = bn->getParentJoint();
  Eigen::Isometry3d tf = p_j->getTransformFromParentBodyNode();
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();

  // TODO: This could be substitute by computeRotation in DART 6.0
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


bool ASFData::generateSkeletonHierarchy(dart::dynamics::SkeletonPtr skel)
{

  std::cout << "start generate skeleton" << std::endl;
  std::istringstream stream;
  std::string line, parentBoneName, childBoneName;
  Bone childBone;
  BodyNodePtr parentBodyNodePtr, childBodyNodePtr;

  // get the begin token, if not, the format is wrong
  std::string token;
  std::getline(mRetriever, line);
  stream.clear();
  stream.str(line);
  stream >> token;
  if (token != "begin")
  {
    std::cout << "Wrong ASF Format: lost begin token at hierarchy section"
              << std::endl;
    return false;
  }

  
  // create root
  parentBodyNodePtr = createRoot(skel, nullptr, &mRoot);

  // attach bones
  while (std::getline(mRetriever, line))
  {
    stream.clear();
    stream.str(line);
    stream >> parentBoneName;
    if (parentBoneName == "end") break;

    parentBodyNodePtr = skel->getBodyNode(parentBoneName);
    while(stream >> childBoneName)
    {
      if (getBone(&childBone, childBoneName))
      {
        childBodyNodePtr = createSegment(skel, parentBodyNodePtr, &childBone);
      }

    }
  }

  std::cout << "finish generating structure" << std::endl;
  return true;
}





bool ASFData::readSkeleton(char * fileName, dart::dynamics::SkeletonPtr skel)
{

  // init retriever
  mRetriever.open(fileName, std::ios::in);
  assert(mRetriever);

  // skip macro
  std::string line, token, dof_val;
  while (std::getline(mRetriever, line))
  {
    std::istringstream stream(line);
    stream >> token;
    if (token == ":root") break;
  }


  // set root data
  setRoot();


  // set Bone data
  setBones();


  // generate the whole skeleton with hierarchy structure
  generateSkeletonHierarchy(skel);


  return true;
}
