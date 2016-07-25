/**
 * Version: skeleton data only
 * Steps
 * 1) fix writing the skeleton kinematic data
 */
#include <dart/dart.hpp>
#include "MyWindow.h"
#include <iostream>
#include <cmath> // for atan2
#include "AsfParser.h"

//#include <GL/glut.h>
//#include "AmcMotion.h"
#include "LinearInterpolator.h"

using namespace dart;
using namespace dynamics;
using namespace simulation;

/*
SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 5.0;
  double floor_height = 0.01;
  dart::dynamics::ShapePtr floor_shape;
  floor_shape.reset(new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  //dart::dynamics::ShapeNode* sn = body
  box->setColor(dart::Color::Black());

  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createPillar()
{
  SkeletonPtr pillar = Skeleton::create("pillar");
  BodyNodePtr bn;

  BallJoint::Properties j_prop;
  j_prop.mName = "pillar_joint";
  j_prop.mPositionLowerLimits = Eigen::Vector3d(
      -DART_DBL_INF,
      0,
      -DART_DBL_INF);
  j_prop.mPositionLowerLimits = Eigen::Vector3d(
      DART_DBL_INF,
      0,
      DART_DBL_INF);

  BodyNode::Properties b_prop;
  b_prop.mName = "pillar";
  bn = pillar->createJointAndBodyNodePair<BallJoint>(
    nullptr, j_prop, b_prop).second;

  // make joint shape
  const double& j_R = 0.2;
  std::shared_ptr<EllipsoidShape> j_shape(
      new EllipsoidShape(sqrt(2)*Eigen::Vector3d(j_R, j_R, j_R)));
  j_shape->setColor(dart::Color::Red());
  bn->addVisualizationShape(j_shape);

  // make lhipjoint body shape
  // Create a CylinderShape to be used for both visualization and
  // collision checking
  std::shared_ptr<CylinderShape> b_shape(
      new CylinderShape(j_R, 1));
  b_shape->setColor(dart::Color::Black());

  // Add it as a visualization and collision shape
  bn->addVisualizationShape(b_shape);


  return pillar;

}
*/

int main(int argc, char ** argv)
{

  // testing asfparser
  ASFData* asf_data = new ASFData();
  dart::dynamics::SkeletonPtr robot = dart::dynamics::Skeleton::create("robot");
  asf_data->readSkeleton(argv[1], robot);

  WorldPtr world(new World);

  world->addSkeleton(robot);
  // make sure the world is not intefered with gravity
  Eigen::Vector3d zero_g = Eigen::Vector3d::Zero();
  world->setGravity(zero_g);


  // Display the result

  //MyWindow window(world);
  MyWindow window;
  window.setWorld(world);
  window.setSkel(robot);
  window.loadAndInterpolateMotionData(argv[2], asf_data);

  std::cout << "asf dofs" << std::endl;
  std::vector<std::string> segmentNames;
  Eigen::VectorXd segmentDofs;
  if (asf_data->getSegmentNames(&segmentNames))
  {
    for (std::size_t i=0; i<segmentNames.size(); ++i)
    {
      if (asf_data->getSegmentDegreeOfFreedoms(segmentNames.at(i), &segmentDofs))
      {
        std::cout << segmentNames.at(i) << std::endl;
        std::cout << segmentDofs << std::endl;
      }
    }
  }



  glutInit(&argc, argv);
  window.initWindow(800, 800, "Linear Motion Interpolation");
  glutMainLoop();
 
  return 0;

}
