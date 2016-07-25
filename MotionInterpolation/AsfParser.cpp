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

ASFData::ASFData(const ASFData & asfData) : mRoot(asfData.mRoot),
                                            mBones(asfData.mBones)
{
}


ASFData& ASFData::operator= (const ASFData & asfData)
{
  mRoot = asfData.mRoot;
  mBones = asfData.mBones;
  return *this;

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

bool ASFData::getSegmentAxes(std::string segmentName,
                             Eigen::Vector3d * segmentAxes)
{
  for (int i=0; i<mBones.size(); ++i)
  {
    if (mBones.at(i).name == segmentName)
    {
      *segmentAxes = mBones.at(i).axes;
      return true;
    }
  }
  return false;

}



bool ASFData::getSegmentDegreeOfFreedoms(std::string segmentName,
                                         Eigen::VectorXd * segmentDofs)
{
  if (segmentName == "root")
  {
    *segmentDofs = Eigen::Vector6d::Ones();
    return true;
  }
  else
  {
    for (int i=0; i<mBones.size(); ++i)
    {
      if (mBones.at(i).name == segmentName)
      {
        *segmentDofs = mBones.at(i).dofs;
        return true;
      }
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


bool ASFData::getSegmentNames(std::vector<std::string> * segmentNameList)
{
  std::vector<std::string> list(0);
  list.push_back("root");
  for (std::size_t i=0; i<mBones.size(); ++i)
  {
    list.push_back(mBones.at(i).name);
  }
  if (list.size() > 0)
  {
    *segmentNameList = list;
    return true;
  }
  else
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
  std::cout << "current mBones size = " << mBones.size();
  std::string line, token, dof_val;
  bool is_seg_begin = false;
  Bone newBone;
  std::istringstream stream;
  double axis_buff; // axis buffer
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
      newBone.dofs = Eigen::Vector3d::Zero(); // start new bone's DOFS
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


      // read length
      if (token == "axis")
      {
        for (int i=0; i<3; ++i) // suppose direction is 3D
        {
          stream >> axis_buff;
          newBone.axes(i) = axis_buff;
        }
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
        else
        {
          newBone.dofs = Eigen::Vector3d(0, 0, 0);
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
    std::cout << ", axes = " << (*it).axes;
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


  FreeJoint::Properties j_prop;
  j_prop.mName = "root_joint";
  BodyNode::Properties b_prop;
  b_prop.mName = "root";
  parent = nullptr;

  // create joint and body node pair
  dart::dynamics::BodyNode* bn = nullptr;

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
    parent, j_prop, b_prop);

  bn = pair.second;
  assert(bn != nullptr);


  // create joint shape node
  const double& j_R = joint_radius;
  dart::dynamics::ShapePtr j_shape;
  j_shape.reset(new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_R, j_R, j_R)));
  dart::dynamics::ShapeNode* sn = bn->createShapeNodeWith<
    dart::dynamics::VisualAspect,
    dart::dynamics::CollisionAspect,
    dart::dynamics::DynamicsAspect>(j_shape, "root_shape");
  sn->getVisualAspect()->setColor(dart::Color::Blue());

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

  // create joint properties pointer
  BallJoint::Properties j_prop;
  j_prop.mName = bone->name + "_joint";
  j_prop.mT_ParentBodyToJoint.translation() = bone->length * unit * bone->direction;


  BodyNode::Properties b_prop;
  b_prop.mName = bone->name;
  // body node and joint should be parent and child relationship
  // e.g., lhipjoint_joint is the joint driven lfemur
  bn = skel->createJointAndBodyNodePair<BallJoint>(
    parent, j_prop, b_prop).second;



/*
  // make parent joint shape
  const double& j_R = joint_radius;
  std::shared_ptr<EllipsoidShape> j_shape(
      new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_R, j_R, j_R)));
  dart::dynamic::shapeNode* node = bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics
  dart::dynamics::ShapeNode* sn = bn->createShapeNodeWith<
    dart::dynamics::VisualAspect,
    dart::dynamics::CollisionAspect,
    dart::dynamics::DynamicsAspect>(j_shape, "root_shape");
  sn->getVisualAspect()->setColor(dart::Color::Blue());


  bn->addVisualizationShape(j_shape);

  j_shape->getVisualAspect()->setColor(dart::Color::Red());
*/

  // Create a CylinderShape to be used for both visualization and
  // collision checking
  dart::dynamics::Joint*    joint  = bn->getParentJoint();
  Eigen::Isometry3d         tf     = joint->getTransformFromParentBodyNode();

  // Determine the local transform of the shape
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
  localTransform.linear() = dart::math::computeRotation(tf.translation(),
                              dart::math::AxisType::AXIS_Z);
  localTransform.translation() = 0.5 * tf.translation();

  const double& b_R = joint_radius;
  dart::dynamics::ShapePtr b_shape;
  b_shape.reset(new CylinderShape(b_R, bone->length * unit));
  dart::dynamics::ShapeNode* sn = parent->createShapeNodeWith<
    dart::dynamics::VisualAspect,
    dart::dynamics::CollisionAspect,
    dart::dynamics::DynamicsAspect>(b_shape, bn->getName()+"_shape");


  sn->setRelativeTransform(localTransform);
  sn->getVisualAspect()->setColor(dart::Color::Black());

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

  // adjust the segment reference (relate to its parent joint) based on axis
  // data
  /*
  JointPtr currentJointPtr;
  JointPtr parentJointPtr;
  BodyNodePtr currentBodyNodePtr;
  std::string currentBodyNodeName;
  Eigen::Vector3d rotationReference;
  Eigen::Isometry3d transformMatrix;
  double deg_to_rad = M_PI/180;
  for (int i=0; i<mBones.size(); ++i)
  {
    currentBodyNodeName = mBones.at(i).name;
    currentJointPtr = skel->getJoint(currentBodyNodeName+"_joint");
    currentBodyNodePtr = skel->getBodyNode(currentBodyNodeName);
    parentJointPtr = currentBodyNodePtr->getParentJoint();

    if (parentJointPtr->getName() != "root")
    {
      rotationReference = mBones.at(i).axes * deg_to_rad;
      //transformMatrix.linear() = rotationReference;
      //parentJointPtr->setTransform(transformMatrix);
      parentJointPtr->setPositions(rotationReference);
    }
  }
  */


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


  // dump lfemur bodynode and its parent reference
  std::cout << "lfemur before motion's relative transform" << std::endl;
  std::cout << skel->getBodyNode("lfemur")->getRelativeTransform().linear() << std::endl;

  std::cout << "lfemur joint positions" << std::endl;
  std::cout << skel->getJoint("lfemur_joint")->getPositions() << std::endl;

  std::cout << "lfemur # body node size = "
            << skel->getBodyNode("lfemur")->getNumChildBodyNodes() << std::endl;


  return true;
}
