#ifndef SPLINE_H
#define SPLINE_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <string>
#include <Eigen/Dense>
extern bool firstLastDeboor;
extern bool firstLastConnect;
extern int PointIndex;
// class for spline
class Spline2d {
public:
    enum SplineType {LINEAR, BEZIER, DEBOOR, CATMULL,CTWO};
public:
    Spline2d();

    void AddPoint(const Eigen::Vector2d&);

    void RemoveLastPoint();
    int CheckPoint(int x, int y);
    void RemoveCertainPoint(int x, int y);
    void RemoveAll();
    void UpdatePointCoor(int i,int x, int y);
    void SetSplineType(SplineType type);

    std::vector<Eigen::Vector2d> GetInterpolatedSpline();

    std::vector<Eigen::Vector2d> GetControlPoints();



private:
    void LinearInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline);

    void BezierInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline);

    void DeBoorInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline);

    void CatmullInterpolate(std::vector<Eigen::Vector2d>& interpolatedSpline);

    void C2Interpolate(std::vector<Eigen::Vector2d>& interpolatedSpline);

    std::vector<Eigen::Vector2d> controlPoints_;
    SplineType type_;
};

#endif  // SPLINE_H
