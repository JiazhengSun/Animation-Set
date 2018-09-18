#include "simulator.h"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(2);

    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;

    mTimeStep = 0.0003;
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

void Simulator::reset() {
    mParticles[0].mPosition[1] = 0.0;
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;

    for (int i = 0; i < getNumParticles(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }

}

void Simulator::simulate() {
    // TODO:
    double d = 0.1;
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;

        MatrixXd J(2,6); MatrixXd Jdot(2,6);
        MatrixXd q(6,1); MatrixXd qdot(6,1);
        MatrixXd Q(6,1); MatrixXd W(6,6);

        Vector3d x1 = mParticles[0].mPosition; Vector3d x2 = mParticles[1].mPosition;
        Vector3d v1 = mParticles[0].mVelocity; Vector3d v2 = mParticles[1].mVelocity;
        Vector3d f1 = mParticles[0].mAccumulatedForce; Vector3d f2 = mParticles[1].mAccumulatedForce;

        MatrixXd lamda(2,1);

        J << x1[0], x1[1], x1[2], 0, 0, 0,
             x1[0]-x2[0], x1[1]-x2[1], x1[2]-x2[2],x2[0]-x1[0],x2[1]-x1[1],x2[2]-x1[2];

        Jdot << v1[0], v1[1], v1[2], 0, 0, 0,
                v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2],v2[0]-v1[0],v2[1]-v1[1],v2[2]-v1[2];

        q << x1[0], x1[1], x1[2],x2[0], x2[1], x2[2];
        qdot << v1[0], v1[1], v1[2], v2[0], v2[1], v2[2];
        Q << f1[0], f1[1], f1[2],f2[0], f2[1], f2[2];

        double m1 = mParticles[0].mMass; double m2 = mParticles[1].mMass;
        W << 1/m1, 0, 0, 0, 0, 0,
             0, 1/m1, 0, 0, 0, 0,
             0, 0, 1/m1, 0, 0, 0,
             0, 0, 0, 1/m2, 0, 0,
             0, 0, 0, 0, 1/m2, 0,
             0, 0, 0, 0, 0, 1/m2;

        //Feed back calculation
        double C1 = (0.5*x1).dot(x1) - 0.5*0.2*0.2;
        double C1dot = x1.dot(v1);
        double C2 = (0.5*(x1-x2)).dot(x1-x2) - 0.5*d*d;
        double C2dot = (x1-x2).dot(v1-v2);
        Vector2d Cf; Cf<<C1, C2;
        Vector2d Cs; Cs<<C1dot, C2dot;

        double ks = 0.5; double kd = 0.5;
        Eigen::MatrixXd A;
        A = J*W*J.transpose();
        Eigen::MatrixXd b;
        b = -1*Jdot*qdot-J*W*Q -ks*Cf - kd*Cs;
        lamda = A.fullPivLu().solve(b);

        MatrixXd Qhat(6,1);
        Qhat = J.transpose() * lamda;

        Vector3d force_one; force_one << Qhat(0,0), Qhat(1,0), Qhat(2,0);
        Vector3d force_two; force_two << Qhat(3,0), Qhat(4,0), Qhat(5,0);

        mParticles[0].mAccumulatedForce += force_one;
        mParticles[1].mAccumulatedForce += force_two;

    }
    //
    for (int i = 0; i < mParticles.size(); i++) {
        //Explicit Euler
        //mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep;
        //mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;

        //MidPoint Method
        Vector3d pos_first_deriv = mParticles[i].mVelocity;
        Vector3d vel_first_deriv = mParticles[i].mAccumulatedForce / mParticles[i].mMass;
        Vector3d x_half, v_half;
        x_half = mParticles[i].mPosition + pos_first_deriv * (mTimeStep/2);
        v_half = mParticles[i].mVelocity + vel_first_deriv * (mTimeStep/2);
        Vector3d x_half_deriv, v_half_deriv;
        x_half_deriv = v_half;
        v_half_deriv = mParticles[i].mAccumulatedForce / mParticles[i].mMass;
        mParticles[i].mPosition += x_half_deriv * mTimeStep;
        mParticles[i].mVelocity += v_half_deriv * mTimeStep;

    }
    //
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }

}
