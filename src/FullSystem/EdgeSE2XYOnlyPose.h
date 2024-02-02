#ifndef EDGE_SE2_XY_ONLY_POSE_H
#define EDGE_SE2_XY_ONLY_POSE_H

// g2o
#include <g2o/types/slam2d/types_slam2d.h> // vertex type
#include <g2o/core/factory.h>
#include "g2o/core/base_binary_edge.h"
#include <Eigen/Geometry>
#include "g2o/core/base_vertex.h"


namespace g2o{

class EdgeSE2XYOnlyPose : public BaseUnaryEdge<2, Eigen::Vector2d, VertexSE2>{



public:    
    EdgeSE2XYOnlyPose(){}

    void computeError();
    virtual void linearizeOplus();
    
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    
    g2o::VertexPointXY* l2;

};
} // namespace g2o

#endif