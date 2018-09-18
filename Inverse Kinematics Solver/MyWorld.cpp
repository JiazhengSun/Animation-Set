#include "MyWorld.h"
#include <iostream>
#include <algorithm>
using namespace Eigen;
using namespace dart::dynamics;
using namespace std;

MyWorld::MyWorld() {
  // Load a skeleton from file
  mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");

  // Create markers
  createMarkers();

  // Initialize Jacobian assuming that there is only one constraint
  // mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
  // for (int i = 0; i < mConstrainedMarker.size(); i++) {
  //   mConstrainedMarker[i] = -1;
  //   //mMarkers[i] = NULL;
  // }
  // //mConstrainedMarker = -1;
  // //mTarget = something?;
}

MyWorld::~MyWorld() {
}

void MyWorld::solve() {
  if (mConstrainedMarker.size() == 0)
    return;
  int numIter = 300;
  double alpha = 0.01;
  int nDof = mSkel->getNumDofs();
  //VectorXd gradients(nDof);
  std::vector<Eigen::VectorXd> gradients;
  VectorXd newPose(nDof);
  for (int i = 0; i < numIter; i++) {
    gradients = updateGradients();
    for(int j = 0; j < gradients.size(); j++) {
      newPose = mSkel->getPositions() - alpha * gradients[j];
      mSkel->setPositions(newPose);
      mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
    }
  }
}

// Current code only works for the left leg with only one constraint
std::vector<Eigen::VectorXd> MyWorld::updateGradients() {

  std::vector<Eigen::VectorXd> gradientArray;
  // compute c(q)
  for (int k = 0; k < mConstrainedMarker.size(); k++) {
    VectorXd gradients;
    mC = getMarker(mConstrainedMarker[k])->getWorldPosition() - mTarget[k];
    mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
    // compute J(q)
    Vector4d offset;
    offset << getMarker(mConstrainedMarker[k])->getLocalPosition(), 1; // Create a vector in homogeneous coordinates
    BodyNode *node = getMarker(mConstrainedMarker[k])->getBodyNode();
    BodyNode *root = mSkel->getRootBodyNode();
    while(node != root) {
      Joint *joint = node->getParentJoint();
      int nDofs = joint->getNumDofs();
      Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
      Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
      Matrix4d jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
      Vector4d jCol; jCol << 0,0,0,0;
      for (int i = 0; i < nDofs; i++) {
        MatrixXd R1, R2;
        R1.setIdentity(4,4); R2.setIdentity(4,4);
        Matrix4d dR = joint->getTransformDerivative(i);
        for (int j = 0; j < i; j++) {
          R1 *= joint->getTransform(j).matrix();
        }
        for (int j = i+1; j < nDofs; j++) {
          R2 *= joint->getTransform(j).matrix();
        }
        jCol = worldToParent * parentToJoint * R1 * dR * R2 * jointToChild * offset;
        int colIndex = joint->getIndexInSkeleton(i);
        mJ.col(colIndex) = jCol.head(3);
      }
      //Update offset
      MatrixXd temp; temp.setIdentity(4,4);
      for (int i = 0; i < nDofs; i++) {
        temp = temp * joint->getTransform(i).matrix();
      }
      offset = parentToJoint * temp * jointToChild * offset;
      node = node->getParentBodyNode();
    }
    gradients = 2 * mJ.transpose() * mC;
    gradientArray.push_back(gradients);
  }
  return gradientArray;
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index) {
  bool flag = false;
  int duplicate;
  for (int i = 0; i < mConstrainedMarker.size(); i++) {
      if (mConstrainedMarker[i] == _index && flag == false) {
        duplicate = i;
        flag = true;
      }
  }
  if (flag == false) { //No duplicate, just push back
    mConstrainedMarker.push_back(_index);
    mTarget.push_back(getMarker(_index)->getWorldPosition());
  }
}

int MyWorld::getIndexOfMarker(int _index) {
    for (int i = 0; i < mConstrainedMarker.size(); i++) {
      if (mConstrainedMarker[i] == _index) {
        return i;
      }
    }
}

void MyWorld::modifyConstraint(Vector3d _deltaP, int _index) {
  int temp;
  temp = getIndexOfMarker(_index);
  mTarget[temp] += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
  int temp = getIndexOfMarker(_index);
  if (temp < mConstrainedMarker.size()) {
    mConstrainedMarker.erase(mConstrainedMarker.begin() + temp);
    mTarget.erase(mTarget.begin() + temp);
  }
}

Marker* MyWorld::getMarker(int _index) {

  return mMarkers[_index];
}

void MyWorld::createMarkers() {
  Vector3d offset(0.2, 0.0, 0.0);
  BodyNode* bNode = mSkel->getBodyNode("h_heel_right");
  Marker* m = new Marker("right_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.2, 0.0, 0.0);
  bNode = mSkel->getBodyNode("h_heel_left");
  m = new Marker("left_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_right");
  m = new Marker("right_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_left");
  m = new Marker("left_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, 0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, -0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.075, 0.1, 0.0);
  bNode = mSkel->getBodyNode("h_abdomen");
  m = new Marker("abdomen", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, 0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, -0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_right");
  m = new Marker("right_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_left");
  m = new Marker("left_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, 0.05);
  bNode = mSkel->getBodyNode("h_bicep_right");
  m = new Marker("right_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, -0.05);
  bNode = mSkel->getBodyNode("h_bicep_left");
  m = new Marker("left_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, 0.025);
  bNode = mSkel->getBodyNode("h_hand_right");
  m = new Marker("right_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, -0.025);
  bNode = mSkel->getBodyNode("h_hand_left");
  m = new Marker("left_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);
}
