#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(4);

    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.35;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = -0.15;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.15;
    mParticles[2].mPosition[1] = 20.0;

    mParticles[3].mPosition[0] = 0.35;
    mParticles[3].mPosition[1] = 20.0;

    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[3].mColor = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);

    mTimeStep = 0.03;
    mElapsedTime = 0;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::initialVelocity(){
  float velocity;
  cout<<"Please enter a value as initial velocity. Press enter when you are done. Then run simulation in the window."<<endl;
  cin>>velocity;
  setInitialVelocity(velocity);
}

void Simulator::reset() {
    mParticles[0].mPosition[0] = -0.35;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = -0.15;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.15;
    mParticles[2].mPosition[1] = 20.0;

    mParticles[3].mPosition[0] = 0.35;
    mParticles[3].mPosition[1] = 20.0;

    for (int i = 0; i < 4; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    mElapsedTime = 0;
}

void Simulator::setInitialVelocity(float vel) {
  for(int i = 0; i < 4; i+=1) {
    mParticles[i].mVelocity[1] = vel;
  }
}


void Simulator::simulate() {

    //Red ball x += vot + 1/2 gt^2
    mParticles[0].mPosition[1] += mParticles[0].mVelocity[1] * mTimeStep - 0.5 * 9.8 * mTimeStep * mTimeStep;
    mParticles[0].mVelocity[1] += mTimeStep * -9.8;

    //First Blue Ball Explicit Euler method
    float oldV = mParticles[1].mVelocity[1];
    mParticles[1].mAccumulatedForce[1] = mParticles[1].mMass * -9.8;
    mParticles[1].mVelocity[1] += mTimeStep * (mParticles[1].mAccumulatedForce[1] / mParticles[1].mMass);
    mParticles[1].mPosition[1] += mTimeStep * oldV;

    //Second Blue Ball mid point method
    mParticles[2].mAccumulatedForce[1] = mParticles[2].mMass * -9.8;
    float deltaX_1 = mTimeStep * mParticles[2].mVelocity[0];
    float deltaX_2 = mParticles[2].mAccumulatedForce[1] / mParticles[2].mMass; //Step one

    float fmid_1 = mParticles[2].mPosition[1] + 0.5 * mTimeStep * mParticles[2].mVelocity[1];
    float fmid_2 = mParticles[2].mVelocity[1] - 0.5 * mTimeStep * 9.8;

    float dmid_1 = mParticles[2].mVelocity[1] - 0.5 * mTimeStep * 9.8;
    float dmid_2 = -9.8;

    mParticles[2].mPosition[1] += mTimeStep * dmid_1;
    mParticles[2].mVelocity[1] += mTimeStep * dmid_2;

    //Thrid Ball use semi-implicit
    mParticles[3].mVelocity[1] -= 9.8 * mTimeStep;
    mParticles[3].mPosition[1] += mTimeStep * mParticles[3].mVelocity[1];

    mElapsedTime += mTimeStep;
}
