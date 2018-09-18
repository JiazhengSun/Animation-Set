#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>
#include <cmath>
using namespace Eigen;
using namespace std;

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();
    // Create and intialize two default rigid bodies
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -0.3;
    rb1->mPosition[1] = -0.5;

    rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    mRigidBodies.push_back(rb1);

    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
}
void MyWorld::addMoreBalls() {
    RigidBody *rb =  new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb, "ellipse"); // Put rb2 in collision detector
    rb->mPosition[0] = (((double) rand() / (RAND_MAX))-0.5)*0.8;
    rb->mPosition[1] = -((double) rand() / (RAND_MAX))-0.2;
    rb->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb->mColor = Vector4d(((double) rand() / (RAND_MAX)), ((double) rand() / (RAND_MAX)), ((double) rand() / (RAND_MAX)), 1.0); // Blue
    mRigidBodies.push_back(rb);
}

void MyWorld::initializePinata() {
    // Add pinata to the collison detector
    mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));

    // Add some damping in the Pinata joints
    int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
    for (int i = 0; i < nJoints; i++) {
        int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
        for (int j = 0; j < nDofs; j++)
        mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
    }

    // Weld two seems to make a box
    dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
    dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
    dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
    dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);
    dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);
    mPinataWorld->getConstraintSolver()->addConstraint(joint1);
    mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
    delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
    delete mCollisionDetector;
}

void MyWorld::simulate() {
    mFrame++;
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass; //linear velocity
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;//force
        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;
    }

    // Deal with orientation and angular mAngMomentum
    // Eigen::Matrix3d mOrientation;Eigen::Vector3d mAngMomentum;Eigen::Quaterniond mQuatOrient;
    for (int i = 0; i < mRigidBodies.size(); i++) {
      Eigen::Matrix3d R = mRigidBodies[i]->mOrientation;
      Eigen::Matrix3d Itensor = R *  mRigidBodies[i]->mInertia * R.transpose();
      Eigen::Vector3d dOri = Itensor.inverse() * mRigidBodies[i]->mAngMomentum; //angular velocity

      Eigen::Quaterniond spin, dq;
      spin.w() = 0;
      spin.vec() = dOri * 0.5;

      dq.w() = -1.0 * (spin.vec().dot(mRigidBodies[i]->mQuatOrient.vec()));
      dq.vec() = mRigidBodies[i]->mQuatOrient.w() * spin.vec() + spin.vec().cross(mRigidBodies[i]->mQuatOrient.vec());
      dq.normalize();

      mRigidBodies[i]->mQuatOrient.w() += dq.w() * mTimeStep;
      mRigidBodies[i]->mQuatOrient.vec() += dq.vec() * mTimeStep;
      mRigidBodies[i]->mQuatOrient.normalize();
      mRigidBodies[i]->mOrientation = mRigidBodies[i]->mQuatOrient.toRotationMatrix();
      mRigidBodies[i]->mAngMomentum += mTimeStep * mRigidBodies[i]->mAccumulatedTorque;
    }

    // Reset accumulated force and torque to be zero after a complete integration
    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mAccumulatedForce.setZero();
        mRigidBodies[i]->mAccumulatedTorque.setZero();
    }

    // Apply external force to the pinata
    mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
    mForce.setZero();

    // Simulate Pinata using DART
    mPinataWorld->step();

    // Run collision detector
    mCollisionDetector->checkCollision();

    // TODO: implement a collision handler
    collisionHandling();

    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;
    int nContacts = mCollisionDetector->getNumContacts();
    for (int i = 0; i < nContacts; i++) {
      Vector3d point = mCollisionDetector->getContact(i).point;
      RigidBody* A = mCollisionDetector->getContact(i).rb1;
      RigidBody* B = mCollisionDetector->getContact(i).rb2;
      Vector3d normal = mCollisionDetector->getContact(i).normal;
      Vector3d padot, pbdot, ra, rb;
      Matrix3d aItensorInv, bItensorInv;
      double vr, ma, mb;

      if (A == nullptr) { //A is Pinata
        padot = mCollisionDetector->getContact(i).pinataVelocity;
        ma = 0;
        aItensorInv <<0,0,0,
                  0,0,0,
                  0,0,0;
        ra <<0,0,0;
      }
      else { //A is rigidbody
        Vector3d va = A->mLinMomentum / A->mMass;
        Matrix3d R = A->mOrientation;
        Matrix3d aItensor = R * A->mInertia * R.transpose();
        aItensorInv = aItensor.inverse();
        Vector3d wa = aItensorInv * A->mAngMomentum;
        ra = point - A->mPosition;
        padot = va + wa.cross(ra);
        ma = 1/A->mMass;
      }

      if (B == nullptr) { //B is pinata
        pbdot = mCollisionDetector->getContact(i).pinataVelocity;
        mb = 0;
        bItensorInv<<0,0,0,
                  0,0,0,
                  0,0,0;
        rb<<0,0,0;
      }
      else { //B is rigidbody
        Vector3d vb = B->mLinMomentum / B->mMass;
        Matrix3d R = B->mOrientation;
        Matrix3d bItensor = R *  B->mInertia * R.transpose();
        bItensorInv = bItensor.inverse();
        Vector3d wb = bItensorInv * B->mAngMomentum;
        rb = point - B->mPosition;
        pbdot = vb + wb.cross(rb);
        mb = 1/B->mMass;
      }

      vr = normal.dot(padot - pbdot);
      if (vr < 0) {
        double topPart = -1.0*(1.0+epsilon)*vr;
        double massInv = ma + mb;
        double downAPart = normal.dot((aItensorInv * (ra.cross(normal))).cross(ra));
        double downBPart = normal.dot((bItensorInv * (rb.cross(normal))).cross(rb));
        double j = topPart / (massInv + downAPart + downBPart);
        Vector3d JA = j*normal;
        Vector3d JB = -j*normal;
        Vector3d TaoA = ra.cross(JA);
        Vector3d TaoB = rb.cross(JB);
        if (A != nullptr) {
          //cout<<JA<<endl;
          A->mLinMomentum += JA;
          A->mAngMomentum += TaoA;
        }
        if (B != nullptr) {
          B->mLinMomentum += JB;
          B->mAngMomentum += TaoB;
        }
      }
    }
}
