#ifndef EDGE_SELF_DEFINED_H
#define EDGE_SELF_DEFINED_H

// g2o
#include <g2o/types/slam2d/types_slam2d.h> // vertex type
#include <g2o/core/factory.h>
#include <Eigen/Geometry>
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"
#include <g2o/types/slam3d/types_slam3d.h>

#include <iostream>
#include <fstream>
#include <time.h>
#include <chrono>

namespace g2o
{

class EdgePointXYOnly : public BaseUnaryEdge<2, Eigen::Vector2d, VertexPointXY>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgePointXYOnly();

    virtual void computeError();

    void setSE2(const SE2 T);
    void setVelocity(const Eigen::Vector3d v);
    void setMeasurement(const Eigen::Vector2d m);
    void setDeltaT(const double deltaT);

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

protected:
    SE2 T0w;
    Vector3 velocity;
    double _deltaT;
};

/* Edge to correlate velocity and SE2, Point is fixed here */
class EdgeSE2VelocityPointFixed : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXYZ>
// class EdgeSE2VelocityPointFixed : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE2VelocityPointFixed();
    ~EdgeSE2VelocityPointFixed();

    virtual void computeError();
    virtual void linearizeOplus();

    void log(int current_frame_id, int point_id, int frame_id);
    void setPointWorldPos(const Eigen::Vector2d _Pw);
    void setMeasurement(const Eigen::Vector2d m);
    void setDeltaT(const double deltaT);
    void setVelocityZ(const double v_z);

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

protected:
    Eigen::Vector2d Pw;
    double _deltaT;
    double _velocityZ;

    std::ofstream csvFile;
    int iteration;
    int pointId;
};

/* Edge to correlate Point, velocity and SE2*/
class EdgeSE2PointXYVelocity : public BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE2PointXYVelocity();
    ~EdgeSE2PointXYVelocity();

    void log(int point_id, int frame_id);

    virtual void computeError();
    virtual void linearizeOplus();

    void setMeasurement(const Eigen::Vector2d m);
    void setDeltaT(const double deltaT);

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

protected:
    double _deltaT;

    std::ofstream csvFile;
    int iteration;
};

/**
 * EdgeVelocity only
 */
class EdgeVelocityOnly : public BaseUnaryEdge<3, Eigen::Vector3d, VertexPointXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeVelocityOnly();

    virtual void computeError();
    virtual void linearizeOplus();

    void setMeasurement(const Eigen::Vector3d m);
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;
};

/**
 * Edge for Velocity and Pose Correlation
*/
// class EdgeVelocitySE2 : public BaseMultiEdge<3, Eigen::Vector3d>
// class EdgeVelocitySE2 : public BaseMultiEdge<2, Eigen::Vector2d>
// class EdgeVelocitySE2 : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXYZ>
// class EdgeVelocitySE2 : public BaseBinaryEdge<1, double, VertexSE2, VertexPointXYZ>
class EdgeVelocitySE2 : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE2, VertexPointXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeVelocitySE2();

    virtual void computeError();
    // virtual void linearizeOplus();

    void log(int frame_id);

    void setDeltaT(const double deltaT);

    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    VertexSE2 *vj;
protected:
    double _deltaT;
    std::ofstream csvFile;
    int iteration;
};

} // namespace g2o

#endif