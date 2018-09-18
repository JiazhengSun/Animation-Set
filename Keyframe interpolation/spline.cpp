#include "spline.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace Eigen;
//bool firstLastDeboor;

Spline2d::Spline2d() {
    controlPoints_.clear();
    type_ = Spline2d::LINEAR;
}

void Spline2d::AddPoint(const Eigen::Vector2d& pt) {
    controlPoints_.push_back(pt);
}

void Spline2d::RemoveLastPoint() {
    if (controlPoints_.size() != 0) {
        controlPoints_.erase(controlPoints_.end()-1);
    }
}

void Spline2d::RemoveCertainPoint(int x, int y) {
  controlPoints_.erase(controlPoints_.begin() + CheckPoint(x, y));
}

int Spline2d::CheckPoint(int x, int y) {
  for (int i = 0; i < controlPoints_.size(); i+=1) {
    float xDiff = abs(controlPoints_[i][0] - x);
    float yDiff = abs(controlPoints_[i][1] - y);
    if (xDiff < 30 && yDiff < 30) {
      return (i);
    }
  }
  return (-1);
}

void Spline2d::UpdatePointCoor(int i,int x, int y){
  controlPoints_[i][0] = x;
  controlPoints_[i][1] = y;
}

void Spline2d::RemoveAll() {
    controlPoints_.clear();
}

void Spline2d::SetSplineType(Spline2d::SplineType type) {
    type_ = type;
}

vector<Eigen::Vector2d> Spline2d::GetInterpolatedSpline() {
    vector<Eigen::Vector2d> interpolatedSpline;
    interpolatedSpline.clear();

    if (type_ == Spline2d::LINEAR) {
        LinearInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::BEZIER) {
        BezierInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::DEBOOR) {
        DeBoorInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::CATMULL) {
        CatmullInterpolate(interpolatedSpline);
    } else if (type_ == Spline2d::CTWO) {
        C2Interpolate(interpolatedSpline);
    }

    return interpolatedSpline;
}

vector<Eigen::Vector2d> Spline2d::GetControlPoints() {
    return controlPoints_;
}

void Spline2d::LinearInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline) {
    interpolatedSpline = controlPoints_;
}

void Spline2d::BezierInterpolate(vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for bezier interpolation
    Matrix4f base_matrix;
    base_matrix << -1, 3, -3, 1,
                   3, -6, 3, 0,
                   -3, 3, 0, 0,
                   1, 0, 0, 0;
    if(firstLastConnect == true) {
      controlPoints_.push_back(controlPoints_[0]);
      controlPoints_.push_back(controlPoints_[0]);
      controlPoints_.push_back(controlPoints_[0]);
    }
    int points_number = controlPoints_.size();
    if (points_number < 4) {
      //Do Nothing here
    } else {
      for(int i = 0; i < points_number- 3; i+=3) {
        MatrixXf pxVector(4,1);
        MatrixXf pyVector(4,1);
        pxVector << controlPoints_[i][0],controlPoints_[i+1][0],controlPoints_[i+2][0],controlPoints_[i+3][0];
        pyVector << controlPoints_[i][1],controlPoints_[i+1][1],controlPoints_[i+2][1],controlPoints_[i+3][1];
        for (float t = 0; t < 1; t += 0.001) {
          MatrixXf tVector(1,4);
          tVector << t*t*t, t*t, t, 1;
          MatrixXf newX = tVector * base_matrix * pxVector;
          MatrixXf newY = tVector * base_matrix * pyVector;
          Vector2d pointVec(newX(0), newY(0));
          interpolatedSpline.push_back(pointVec);
        }
      }
    }
    if (firstLastConnect == true) {
      controlPoints_.erase(controlPoints_.end()-1);
      controlPoints_.erase(controlPoints_.end()-1);
      controlPoints_.erase(controlPoints_.end()-1);
    }

}

void Spline2d::DeBoorInterpolate(vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for deboor interpolation
    Matrix4f base_matrix;
    base_matrix << -1, 3, -3, 1,
                   3, -6, 3, 0,
                   -3, 0, 3, 0,
                   1, 4, 1, 0;
    base_matrix *= 1/6.0;

    if(firstLastDeboor == true){
      controlPoints_.insert(controlPoints_.begin(), controlPoints_[0]);
      controlPoints_.insert(controlPoints_.begin(), controlPoints_[0]);
      controlPoints_.insert(controlPoints_.end()-1, controlPoints_.back());
      controlPoints_.insert(controlPoints_.end()-1, controlPoints_.back());
    }
    if(firstLastConnect == true) {
      controlPoints_.push_back(controlPoints_[0]);
      controlPoints_.push_back(controlPoints_[1]);
      controlPoints_.push_back(controlPoints_[2]);
    }
    int points_number = controlPoints_.size();
    for (int i = 0; i < points_number-3; i+=1) {
      MatrixXf pxVector(4,1);
      MatrixXf pyVector(4,1);
      pxVector << controlPoints_[i][0],controlPoints_[i+1][0],controlPoints_[i+2][0],controlPoints_[i+3][0];
      pyVector << controlPoints_[i][1],controlPoints_[i+1][1],controlPoints_[i+2][1],controlPoints_[i+3][1];
      for (float t = 0; t < 1; t += 0.001) {
        MatrixXf tVector(1,4);
        tVector << t*t*t, t*t, t, 1;
        MatrixXf newX = tVector * base_matrix * pxVector;
        MatrixXf newY = tVector * base_matrix * pyVector;
        Vector2d pointVec(newX(0), newY(0));
        interpolatedSpline.push_back(pointVec);
      }
    }
    if(firstLastDeboor == true) {
      controlPoints_.erase(controlPoints_.begin());controlPoints_.erase(controlPoints_.begin());
      controlPoints_.erase(controlPoints_.end()-1);controlPoints_.erase(controlPoints_.end()-1);
    }
    if (firstLastConnect == true) {
      controlPoints_.erase(controlPoints_.end()-1);
      controlPoints_.erase(controlPoints_.end()-1);
      controlPoints_.erase(controlPoints_.end()-1);
    }

}

void Spline2d::CatmullInterpolate(vector<Eigen::Vector2d>& interpolatedSpline) {
    // TODO: fill in code for catmull-rom interpolation
    Matrix4f base_matrix;
    base_matrix << 2, -2, 1, 1,
                   -3, 3, -2, -1,
                   0, 0, 1, 0,
                   1, 0, 0, 0;
    if (firstLastConnect == true) {
      controlPoints_.push_back(controlPoints_[0]);
    }
    int points_number = controlPoints_.size();
    for (int i = 0; i < points_number - 1; i+=1) {
      int j = i;
      int d1[2]; int dn[2];
      float d1x; float d1y; float dnx; float dny;

      if (j == 0){ //D0 = C1 - C0
        d1x = controlPoints_[j+1][0]-controlPoints_[j][0];
        d1y = controlPoints_[j+1][1]-controlPoints_[j][1];
      } else { //1/2 * (C2-C0)
        d1x = 0.5*(controlPoints_[j+1][0]-controlPoints_[j-1][0]);
        d1y = 0.5*(controlPoints_[j+1][1]-controlPoints_[j-1][1]);
      }
      d1[0] = d1x; d1[1] = d1y;

      j += 1;
      if (j == points_number - 1) { //Cn = Cn-1
        dnx = controlPoints_[j][0]-controlPoints_[j-1][0];
        dny = controlPoints_[j][1]-controlPoints_[j-1][1];
      } else {
        dnx = 0.5*(controlPoints_[j+1][0]-controlPoints_[j-1][0]);
        dny = 0.5*(controlPoints_[j+1][1]-controlPoints_[j-1][1]);
      }
      dn[0] = dnx; dn[1] = dny;
      MatrixXf pxVector(4,1);
      MatrixXf pyVector(4,1);
      pxVector << controlPoints_[i][0],controlPoints_[i+1][0],d1[0],dn[0];
      pyVector << controlPoints_[i][1],controlPoints_[i+1][1],d1[1],dn[1];
      for (float t = 0; t < 1; t += 0.001) {
          MatrixXf tVector(1,4);
          tVector << t*t*t, t*t, t, 1;
          MatrixXf newX = tVector * base_matrix * pxVector;
          MatrixXf newY = tVector * base_matrix * pyVector;
          Vector2d pointVec(newX(0), newY(0));
          interpolatedSpline.push_back(pointVec);
        }
    }
    if (firstLastConnect){
      controlPoints_.erase(controlPoints_.end()-1);
    }
}

void Spline2d::C2Interpolate(vector<Eigen::Vector2d>& interpolatedSpline) {
    int num = controlPoints_.size();
    Vector2d *dList = new Vector2d[num];
    float coeList [num];
    for (int i = 0; i < num; i += 1) { // Matrix diagonal coefficients
      if (i == 0) {
        float temp = 2.0;
        coeList[i] = temp;
      } else {
        float temp = 4.0-(1.0/coeList[i-1]);
        coeList[i] = temp;
      }
    }
    for (int i = num-1; i >= 0; i-=1) { // Derative list
      float tempx, tempy;
      if (i == num - 1) {
        tempx = 3*(controlPoints_[i][0] - controlPoints_[i-1][0]) / coeList[i];
        tempy = 3*(controlPoints_[i][1] - controlPoints_[i-1][1]) / coeList[i];
      } else {
        tempx = (3*(controlPoints_[i+1][0] - controlPoints_[i-1][0]) - dList[i+1][0]) / coeList[i];
        tempy = (3*(controlPoints_[i+1][1] - controlPoints_[i-1][1]) - dList[i+1][1]) / coeList[i];
      }
      Vector2d tempVec;
      tempVec << tempx, tempy;
      dList[i] = tempVec;
    }
    for (int i = 0; i < num- 1; i += 1) { //Find v0-v3
        float v0x = controlPoints_[i][0]; float v0y = controlPoints_[i][1];
        float v1x = controlPoints_[i][0] + dList[i](0)/3; float v1y = controlPoints_[i][1] + dList[i](1)/3;
        float v2x = controlPoints_[i+1][0] - dList[i+1](0)/3; float v2y = controlPoints_[i+1][1] - dList[i+1](1)/3;
        float v3x = controlPoints_[i+1][0]; float v3y = controlPoints_[i+1][1];
        // Bezier at this points
        Matrix4f base_matrix;
        base_matrix << -1, 3, -3, 1,
                       3, -6, 3, 0,
                       -3, 3, 0, 0,
                       1, 0, 0, 0;
         MatrixXf pxVector(4,1);
         MatrixXf pyVector(4,1);
         pxVector << v0x, v1x, v2x, v3x;
         pyVector << v0y, v1y, v2y, v3y;
         for (float t = 0; t < 1; t += 0.001) {
           MatrixXf tVector(1,4);
           tVector << t*t*t, t*t, t, 1;
           MatrixXf newX = tVector * base_matrix * pxVector;
           MatrixXf newY = tVector * base_matrix * pyVector;
           Vector2d pointVec(newX(0), newY(0));
           interpolatedSpline.push_back(pointVec);
         }
    }

}
