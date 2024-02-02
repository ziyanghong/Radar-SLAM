#include "EdgeSE2XYOnlyPose.h"

namespace g2o
{

void EdgeSE2XYOnlyPose::computeError()
{
    const g2o::VertexSE2 *v1 = static_cast<const g2o::VertexSE2 *>(_vertices[0]);
    _error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
}

bool EdgeSE2XYOnlyPose::read(std::istream &is)
{
    int paramId;
    is >> paramId;
    if (!setParameterId(0, paramId))
        return false;
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
    information()(1, 0) = information()(0, 1);
    return true;
}

bool EdgeSE2XYOnlyPose::write(std::ostream &os) const
{
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
    return os.good();
}

void EdgeSE2XYOnlyPose::linearizeOplus()
{
    const g2o::VertexSE2 *vi = static_cast<const g2o::VertexSE2 *>(_vertices[0]);
    const double &x1 = vi->estimate().translation()[0];
    const double &y1 = vi->estimate().translation()[1];
    const double &th1 = vi->estimate().rotation().angle();
    const double &x2 = l2->estimate()[0];
    const double &y2 = l2->estimate()[1];

    double aux_1 = cos(th1);
    double aux_2 = -aux_1;
    double aux_3 = sin(th1);

    _jacobianOplusXi(0, 0) = aux_2;
    _jacobianOplusXi(0, 1) = -aux_3;
    _jacobianOplusXi(0, 2) = aux_1 * y2 - aux_1 * y1 - aux_3 * x2 + aux_3 * x1;
    _jacobianOplusXi(1, 0) = aux_3;
    _jacobianOplusXi(1, 1) = aux_2;
    _jacobianOplusXi(1, 2) = -aux_3 * y2 + aux_3 * y1 - aux_1 * x2 + aux_1 * x1;
}

} // namespace g2o