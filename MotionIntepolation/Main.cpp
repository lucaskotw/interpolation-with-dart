/**
 * Version: skeleton data only
 * Steps
 * 1) fix writing the skeleton kinematic data
 */
#include "dart/dart.h"
#include "MyWindow.h"
#include <iostream>
#include <cmath> // for atan2
#include "AsfParser.h"
#include "AmcMotion.h"

using namespace dart::dynamics;
using namespace dart::simulation;


/*
const double joint_radius = 0.02; // m
const double bone_radius = 0.02; // m



using namespace dart::dynamics;
using namespace dart::simulation;

BodyNodePtr createSegment(
    const SkeletonPtr& skel,
    BodyNodePtr parent,
    const std::string& name,
    const Eigen::Vector3d& dir,
    double len,
    int dof)
{
  // define the unit conversion
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter

  BodyNodePtr bn;

  if (name == "root")
  {
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

  }
  else
  {
    BallJoint::Properties j_prop;
    j_prop.mName = name + "_joint";
    j_prop.mT_ParentBodyToJoint.translation() = len * unit * dir;
    BodyNode::Properties b_prop;
    b_prop.mName = name;
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
        new CylinderShape(j_R, len * unit));
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

  }



  return bn; 
}


int plain_skel_creation(const SkeletonPtr& skel, const std::string& name)
{


    const double& j_R = joint_radius;
  double unit = (1.0/0.45)*2.54/100.0; // scale to inches to meter
  BodyNodePtr r_bn = createSegment(skel, nullptr,
                          "root",
                          Eigen::Vector3d::Zero(),
                          0.0, 7);

  // add lhipjoint
  BodyNodePtr lhip_bn = createSegment(skel, r_bn,
                          "lhipjoint",
                          Eigen::Vector3d(0.61522, -0.761478, 0.204097),
                          2.55743, 7);
 
  // add lfemur
  BodyNodePtr lfe_bn = createSegment(skel, lhip_bn,
                          "lfemur",
                          Eigen::Vector3d(0.34202, -0.939693, 0),
                          7.61162, 7);


  // add ltibia
  BodyNodePtr lti_bn = createSegment(skel, lfe_bn,
                          "ltibia",
                          Eigen::Vector3d(0.34202, -0.939693, 0),
                          8.12393, 7);



  // add lfoot
  BodyNodePtr lfo_bn = createSegment(skel, lti_bn,
                          "lfoot",
                          Eigen::Vector3d(0.0841971, -0.23133, 0.969225),
                          2.3572, 7);


  // add ltoes
  BodyNodePtr ltoes_bn = createSegment(skel, lfo_bn,
                          "ltoes",
                          Eigen::Vector3d(0, 0, 1),
                          1.17993, 7);


  // add rhipjoint
  BodyNodePtr rhip_bn = createSegment(skel, r_bn,
                          "rhipjoint",
                          Eigen::Vector3d(-0.625247, -0.75382, 0.202044),
                          2.58341, 7);


  // add rfemur
  BodyNodePtr rfe_bn = createSegment(skel, rhip_bn,
                          "rfemur",
                          Eigen::Vector3d(-0.34202, -0.939693, 0),
                          7.42147, 7);


  // add rtibia
  BodyNodePtr rti_bn = createSegment(skel, rfe_bn,
                          "rtibia",
                          Eigen::Vector3d(-0.34202, -0.939693, 0),
                          8.20893, 7);



  // add rfoot
  BodyNodePtr rfo_bn = createSegment(skel, rti_bn,
                          "rfoot",
                          Eigen::Vector3d(-0.124331, -0.341598, 0.931586),
                          2.45604, 7);

  // add rtoes
  BodyNodePtr rtoes_bn = createSegment(skel, rfo_bn,
                          "rtoes",
                          Eigen::Vector3d(0, 0, 1),
                          1.21323, 7);

  // upperbody
  // add lowerback
  BodyNodePtr lowb_bn = createSegment(skel, r_bn,
                          "lowerback",
                          Eigen::Vector3d(-0.0218484, 0.994884, -0.0986303),
                          2.00523, 7);


  // add upperback
  BodyNodePtr uppb_bn = createSegment(skel, lowb_bn,
                          "upperback",
                          Eigen::Vector3d(-0.0097305, 0.999717, -0.0217111),
                          2.01352, 7);



  // add thorax
  BodyNodePtr thor_bn = createSegment(skel, uppb_bn,
                          "thorax",
                          Eigen::Vector3d(-0.000928262, 0.999699, -0.0245198),
                          2.02938, 7);

  // add lowerneck
  BodyNodePtr lown_bn = createSegment(skel, thor_bn,
                          "lowerneck",
                          Eigen::Vector3d(-0.0307979, 0.998024, -0.0547665),
                          1.77263, 7);

  // add upperneck
  BodyNodePtr uppn_bn = createSegment(skel, lown_bn,
                          "upperneck",
                          Eigen::Vector3d(0.0531587, 0.996611, -0.0627734),
                          1.77971, 7);

  // add head
  BodyNodePtr head_bn = createSegment(skel, uppn_bn,
                          "head",
                          Eigen::Vector3d(0.0213833, 0.999404, -0.0271147),
                          1.78263, 7);

  // left arm
  // add lclavicle
  BodyNodePtr lcla_bn = createSegment(skel, thor_bn,
                          "lclavicle",
                          Eigen::Vector3d(0.940017, 0.332249, -0.0773233),
                          3.73315, 7);

  // add lhumerus
  BodyNodePtr lhum_bn = createSegment(skel, lcla_bn,
                          "lhumerus",
                          Eigen::Vector3d(1, 0, 0),
                          5.40188, 7);

  // add lradius
  BodyNodePtr lrad_bn = createSegment(skel, lhum_bn,
                          "lradius",
                          Eigen::Vector3d(1, 0, 0),
                          3.68559, 7);

  // add lwrist
  BodyNodePtr lwri_bn = createSegment(skel, lrad_bn,
                          "lwrist",
                          Eigen::Vector3d(1, 0, 0),
                          1.84279, 7);

  // add lhand
  BodyNodePtr lhan_bn = createSegment(skel, lwri_bn,
                          "lhand",
                          Eigen::Vector3d(1, 0, 0),
                          0.61753, 7);

  // add lfingers
  BodyNodePtr lfin_bn = createSegment(skel, lhan_bn,
                          "lfingers",
                          Eigen::Vector3d(1, 0, 0),
                          0.497868, 7);

  // add lthumb
  BodyNodePtr lthu_bn = createSegment(skel, lwri_bn,
                          "lthumb",
                          Eigen::Vector3d(0.707107, 0, 0.707107),
                          0.714842, 7);
  // right arm
  // add rclavicle
  BodyNodePtr rcla_bn = createSegment(skel, thor_bn,
                          "rclavicle",
                          Eigen::Vector3d(-0.9009, 0.425937, -0.0834057),
                          3.27964, 7);
 
  // add rhumerus
  BodyNodePtr rhum_bn = createSegment(skel, rcla_bn,
                          "rhumerus",
                          Eigen::Vector3d(-1, 0, 0),
                          6.1067, 7);
 
  // add rradius
  BodyNodePtr rrad_bn = createSegment(skel, rhum_bn,
                          "rradius",
                          Eigen::Vector3d(-1, 0, 0),
                          3.63052, 7);

  // add rwrist
  BodyNodePtr rwri_bn = createSegment(skel, rrad_bn,
                          "rwrist",
                          Eigen::Vector3d(-1, 0, 0),
                          1.81526, 7);

  // add rhand
  BodyNodePtr rhan_bn = createSegment(skel, rwri_bn,
                          "rhand",
                          Eigen::Vector3d(-1, 0, 0),
                          0.583848, 7);

  // add rfingers
  BodyNodePtr rfin_bn = createSegment(skel, rhan_bn,
                          "rfingers",
                          Eigen::Vector3d(-1, 0, 0),
                          0.470713, 7);

  // add rthumb
  BodyNodePtr rthu_bn = createSegment(skel, rwri_bn,
                          "rthumb",
                          Eigen::Vector3d(-0.707107, 0, 0.707107),
                          0.675852, 7);

  return 0;
}
*/

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 5.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  box->setColor(dart::Color::Black());

  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}



int main(int argc, char ** argv)
{

#if 1 // testing asfparser
  ASFBoneData* asf_data = new ASFBoneData();
  SkeletonPtr skel = asf_data->readSkeleton(argv[1]);

  WorldPtr world(new World);

  SkeletonPtr floor = createFloor();
  world->addSkeleton(skel);
  world->addSkeleton(floor);
   // Display the result
  MyWindow window(world);
  //window.loadMotionData(argv[1]);

  // test motion data
  AmcMotion* motion = new AmcMotion();
  motion->readAMCFile(argv[2]);
  std::vector<std::pair<std::string, std::vector<double>>> motionConfig;
  motionConfig = motion->getMotion(123);
  std::cout << "frame id = 122" << std::endl;
  std::cout << "configuration:" << std::endl;
  for (int i=0; i<motionConfig.size(); ++i)
  {
    std::cout << motionConfig.at(i).first << " ";
    for (int j=0; j<motionConfig.at(i).second.size(); ++j)
      std::cout << motionConfig.at(i).second.at(j) << " ";
    std::cout << std::endl;
  }


  glutInit(&argc, argv);
  window.initWindow(800, 800, "Linear Motion Interpolation");
  glutMainLoop();
 
#else
  // Create the world
  // Create the human skeleton
  // * create skeleton first
  SkeletonPtr skel = Skeleton::create("human");
  SkeletonPtr floor = createFloor();


  // Load ASF data
  // read_asf_file(skel, argv[1]);
  plain_skel_creation(skel, "root");

  // Create a world and add the pendulum to the world
  WorldPtr world(new World);
  

  // Attach skeleton to world
  world->addSkeleton(skel);
  world->addSkeleton(floor);

  // Skeleton configuration
  //std::cout << skel->getState() << std::endl;


  // Display the result
  MyWindow window(world);
  window.loadMotionData(argv[1]);


  glutInit(&argc, argv);
  window.initWindow(800, 800, "CMU Subject 13");
  glutMainLoop();
#endif

  return 0;

}
