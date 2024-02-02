#include "EdgeSelfDefined.h"

namespace g2o
{

/* Edge for optimizing feture only */
EdgePointXYOnly::EdgePointXYOnly() : BaseUnaryEdge<2, Eigen::Vector2d, VertexPointXY>()
{
}

void EdgePointXYOnly::computeError()
{
    VertexPointXY *point = static_cast<VertexPointXY *>(_vertices[0]); // Vertex point
    Eigen::Vector2d Pw = point->estimate();

    SE2 T0t(velocity[0] * _deltaT, velocity[1] * _deltaT, velocity[2] * _deltaT);

    // Error
    _error = T0w.inverse() * Pw - T0t * _measurement; // Estimate - Measurement
}

// Set delta time T respect to T(t=0)
void EdgePointXYOnly::setDeltaT(const double deltaT)
{
    _deltaT = deltaT;
}

void EdgePointXYOnly::setVelocity(const Eigen::Vector3d v)
{
    velocity = v;
}

void EdgePointXYOnly::setSE2(const SE2 T)
{
    T0w = T;
}

void EdgePointXYOnly::setMeasurement(const Eigen::Vector2d m)
{
    _measurement = m; // Local Point p(t=T) observation
}

bool EdgePointXYOnly::read(std::istream &is)
{
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
    information()(1, 0) = information()(0, 1);
    return true;
}

bool EdgePointXYOnly::write(std::ostream &os) const
{
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
    return os.good();
}

/* Edge to correlate velocity and SE2, Point is fixed here */
EdgeSE2VelocityPointFixed::EdgeSE2VelocityPointFixed() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXYZ>()
// EdgeSE2VelocityPointFixed::EdgeSE2VelocityPointFixed() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>()
{}

EdgeSE2VelocityPointFixed ::~EdgeSE2VelocityPointFixed()
{
    csvFile.close();
}

void EdgeSE2VelocityPointFixed::log(int current_frame_id, int point_id, int frame_id)
{
    std::string current_frame_id_str = std::to_string(current_frame_id);

    // const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
    std::string frame_id_str = std::to_string(frame_id);

    // const VertexPointXY *vj = static_cast<const VertexPointXY *>(_vertices[1]);
    std::string point_id_str = std::to_string(point_id);

    std::string log_file = "/home/hong/Desktop/PoseError/EdgeError/EdgeSE2VelocityPointFixed_" +
                           std::string(6 - current_frame_id_str.length(), '0').append(current_frame_id_str) + "_" +
                           std::string(6 - frame_id_str.length(), '0').append(frame_id_str) + "_" +
                           std::string(6 - point_id_str.length(), '0').append(point_id_str) + ".csv";

    csvFile.open(log_file);
    csvFile << "iteration,PointId,T0w.x,T0w.y,T0w.theta,Velocity.x,Velocity.y,Velocity.theta"
            << "PwLocal.x,meas.x,measCorrected.x,error.x,PwLocal.y,meas.y,measCorrected.y,error.y\n";

    iteration = 0;
    pointId = point_id;
}

void EdgeSE2VelocityPointFixed::computeError()
{
    VertexSE2 *T0w = static_cast<VertexSE2 *>(_vertices[0]); // Vertex vehicle pose Ti(t=0) repect to the world
    VertexPointXYZ *velocity = static_cast<VertexPointXYZ *>(_vertices[1]); // Vertex velocity Vi
    // VertexPointXY *velocityXY = static_cast<VertexPointXY *>(_vertices[1]); // Vertex velocity Vi

    Vector3 v = velocity->estimate();
    // Vector3 v;
    // Vector2 vXY = velocityXY->estimate();
    // v[0] = vXY[0];
    // v[1] = vXY[1];
    // v[2] = _velocityZ;
    SE2 T0t(v[0] * _deltaT, v[1] * _deltaT, v[2] * _deltaT);

    // Error
    _error = T0w->estimate().inverse() * Pw - T0t * _measurement; // Estimate - Measurement

    // log
    SE2 se2T0w = T0w->estimate();
    Vector3 vT0w = se2T0w.toVector();
    Vector2 vPwLocal = T0w->estimate().inverse() * Pw;
    Vector2 vMeasCorrected = T0t * _measurement;
    iteration++;
    if (iteration % 2 == 0)
    {
        csvFile << iteration << ","
                << pointId << ","
                << vT0w(0) << ","
                << vT0w(1) << ","
                << vT0w(2) << ","
                << v(0) << ","
                << v(1) << ","
                << v(2) << ","

                << vPwLocal(0) << ","
                << _measurement(0) << ","
                << vMeasCorrected(0) << ","
                << _error(0) << ","

                << vPwLocal(1) << ","
                << _measurement(1) << ","
                << vMeasCorrected(1) << ","
                << _error(1) << "\n";
    }
}

void EdgeSE2VelocityPointFixed::linearizeOplus()
{

    // Jocobian of T0w
    const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
    const double &x1 = vi->estimate().translation()[0];
    const double &y1 = vi->estimate().translation()[1];
    const double &th1 = vi->estimate().rotation().angle();
    const double &x2 = Pw[0];
    const double &y2 = Pw[1];

    double aux_1 = cos(th1);
    double aux_2 = -aux_1;
    double aux_3 = sin(th1);

    _jacobianOplusXi(0, 0) = aux_2;
    _jacobianOplusXi(0, 1) = -aux_3;
    _jacobianOplusXi(0, 2) = aux_1 * y2 - aux_1 * y1 - aux_3 * x2 + aux_3 * x1;
    _jacobianOplusXi(1, 0) = aux_3;
    _jacobianOplusXi(1, 1) = aux_2;
    _jacobianOplusXi(1, 2) = -aux_3 * y2 + aux_3 * y1 - aux_1 * x2 + aux_1 * x1;

    // Jocobian of Velocity
    const VertexPointXYZ *vk = static_cast<const VertexPointXYZ *>(_vertices[1]);
    Vector3 v = vk->estimate();

    double aux_4 = _deltaT;
    double aux_5 = aux_4 * _measurement[0];
    double aux_6 = aux_4 * _measurement[1];
    double aux_7 = v[2] * _deltaT;

    _jacobianOplusXj(0, 0) = -aux_4;
    // _jacobianOplusXj(0, 0) = 0.0;
    _jacobianOplusXj(0, 1) = 0.0;
    _jacobianOplusXj(0, 2) = aux_5 * sin(aux_7) + aux_6 * cos(aux_7);
    // _jacobianOplusXj(0, 2) = 0.0;

    _jacobianOplusXj(1, 0) = 0.0;
    _jacobianOplusXj(1, 1) = -aux_4;
    // _jacobianOplusXj(1, 1) = 0.0;
    _jacobianOplusXj(1, 2) = -aux_5 * cos(aux_7) + aux_6 * sin(aux_7);
    // _jacobianOplusXj(1, 2) = 0.0;

}

// Set delta time T respect to T(t=0)
void EdgeSE2VelocityPointFixed::setDeltaT(const double deltaT)
{
    _deltaT = deltaT;
    // std::cout << "_deltaT: " << _deltaT << std::endl;

}

void EdgeSE2VelocityPointFixed::setVelocityZ(const double v_z)
{
    _velocityZ = v_z;
}

void EdgeSE2VelocityPointFixed::setPointWorldPos(const Eigen::Vector2d _Pw)
{
    Pw = _Pw;
}

void EdgeSE2VelocityPointFixed::setMeasurement(const Eigen::Vector2d m)
{
    _measurement = m; // Point p(t=T) position
}

bool EdgeSE2VelocityPointFixed::read(std::istream &is)
{
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
    information()(1, 0) = information()(0, 1);
    return true;
}

bool EdgeSE2VelocityPointFixed::write(std::ostream &os) const
{
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
    return os.good();
}

/* Edge to correlate Point, velocity and SE2*/
EdgeSE2PointXYVelocity::EdgeSE2PointXYVelocity() : BaseMultiEdge<2, Eigen::Vector2d>()
{
    resize(3);
}

EdgeSE2PointXYVelocity ::~EdgeSE2PointXYVelocity()
{
    csvFile.close();
}

void EdgeSE2PointXYVelocity::log(int point_id, int frame_id)
{
    // const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
    std::string frame_id_str = std::to_string(frame_id);

    // const VertexPointXY *vj = static_cast<const VertexPointXY *>(_vertices[1]);
    std::string point_id_str = std::to_string(point_id);

    std::string log_file = "/home/hong/Desktop/trackPoints/" +
                           std::string(6 - frame_id_str.length(), '0').append(frame_id_str) + "_" +
                           std::string(6 - point_id_str.length(), '0').append(point_id_str) + ".csv";

    csvFile.open(log_file);
    csvFile << "iteration,T0w.x,T0w.y,T0w.theta,T0t.x,T0t.y,T0t.theta,Pw.x,Pw.y,"
            << "PwLocal.x,meas.x,measCorrected.x,error.x,PwLocal.y,meas.y,measCorrected.y,error.y\n";

    iteration = 0;
}

void EdgeSE2PointXYVelocity::computeError()
{
    VertexSE2 *T0w = static_cast<VertexSE2 *>(_vertices[0]);                // Vertex vehicle pose Ti(t=0) repect to the world
    VertexPointXY *Pw = static_cast<VertexPointXY *>(_vertices[1]);         // Vertex Point world coordinate (x,y)
    VertexPointXYZ *velocity = static_cast<VertexPointXYZ *>(_vertices[2]); // Vertex velocity Vi

    Vector3 v = velocity->estimate();
    SE2 T0t(v[0] * _deltaT, v[1] * _deltaT, v[2] * _deltaT);
    // std::cout << "velocity: " << std::endl << velocity->estimate() << std::endl;
    // std::cout << "deltaT: " << _deltaT << std::endl;
    // std::cout << "T0t toVector(): " << std::endl << T0t.toVector() << std::endl << std::endl;
    // std::cout << "_measurement: " << _measurement << std::endl;
    // std::cout << "T0w: " << T0w->estimate() << std::endl;
    // std::cout << "Pw: " << Pw->estimate() << std::endl;

    // Error
    _error = T0w->estimate().inverse() * Pw->estimate() - T0t * _measurement; // Estimate - Measurement

    SE2 se2T0w = T0w->estimate();
    iteration++;
    Vector3 vT0w = se2T0w.toVector();
    Vector2 vPw = Pw->estimate();
    Vector3 vT0t = T0t.toVector();
    Vector2 vPwLocal = T0w->estimate().inverse() * Pw->estimate();
    Vector2 vMeasCorrected = T0t * _measurement;
    csvFile << iteration << ","
            << vT0w(0) << ","
            << vT0w(1) << ","
            << vT0w(2) << ","
            << vT0t(0) << ","
            << vT0t(1) << ","
            << vT0t(2) << ","
            << vPw(0) << ","
            << vPw(1) << ","

            << vPwLocal(0) << ","
            << _measurement(0) << ","
            << vMeasCorrected(0) << ","
            << _error(0) << ","

            << vPwLocal(1) << ","
            << _measurement(1) << ","
            << vMeasCorrected(1) << ","
            << _error(1) << "\n";

    // std::cout << "_error: " <<std::endl << _error << std::endl;
}

void EdgeSE2PointXYVelocity::linearizeOplus()
{

    // auto start1 = std::chrono::high_resolution_clock::now();

    const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexPointXY *vj = static_cast<const VertexPointXY *>(_vertices[1]);
    const double &x1 = vi->estimate().translation()[0];
    const double &y1 = vi->estimate().translation()[1];
    const double &th1 = vi->estimate().rotation().angle();
    const double &x2 = vj->estimate()[0];
    const double &y2 = vj->estimate()[1];

    double aux_1 = cos(th1);
    double aux_2 = -aux_1;
    double aux_3 = sin(th1);

    // Jocobian of T0w
    Eigen::Matrix<double, 2, 3> J_T0w;
    J_T0w(0, 0) = aux_2;
    J_T0w(0, 1) = -aux_3;
    J_T0w(0, 2) = aux_1 * y2 - aux_1 * y1 - aux_3 * x2 + aux_3 * x1;
    J_T0w(1, 0) = aux_3;
    J_T0w(1, 1) = aux_2;
    J_T0w(1, 2) = -aux_3 * y2 + aux_3 * y1 - aux_1 * x2 + aux_1 * x1;
    _jacobianOplus[0] = J_T0w;

    // Jocobian of Pw
    Eigen::Matrix<double, 2, 2> J_Pw;
    J_Pw(0, 0) = aux_1;
    J_Pw(0, 1) = aux_3;
    J_Pw(1, 0) = -aux_3;
    J_Pw(1, 1) = aux_1;
    _jacobianOplus[1] = J_Pw;

    const VertexPointXYZ *vk = static_cast<const VertexPointXYZ *>(_vertices[2]);
    Vector3 v = vk->estimate();

    double aux_4 = _deltaT;
    double aux_5 = aux_4 * _measurement[0];
    double aux_6 = aux_4 * _measurement[1];
    double aux_7 = v[2] * _deltaT;

    // Jocobian of Vi
    Eigen::Matrix<double, 2, 3> J_Vi;
    J_Vi(0, 0) = -aux_4;
    J_Vi(0, 1) = 0.0;
    J_Vi(0, 2) = aux_5 * sin(aux_7) + aux_6 * cos(aux_7);
    J_Vi(1, 0) = 0.0;
    J_Vi(1, 1) = -aux_4;
    J_Vi(1, 2) = -aux_5 * cos(aux_7) + aux_6 * sin(aux_7);
    _jacobianOplus[2] = J_Vi;

    // auto stop1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    // std::cout << "optimization time for linearizeOplus: " << duration1.count() << " ms." << std::endl;
}

bool EdgeSE2PointXYVelocity::read(std::istream &is)
{
    is >> _measurement[0] >> _measurement[1];
    is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
    information()(1, 0) = information()(0, 1);
    return true;
}

bool EdgeSE2PointXYVelocity::write(std::ostream &os) const
{
    os << measurement()[0] << " " << measurement()[1] << " ";
    os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
    return os.good();
}

void EdgeSE2PointXYVelocity::setMeasurement(const Eigen::Vector2d m)
{
    _measurement = m; // Point p(t=T) position
}

// Set delta time T respect to T(t=0)
void EdgeSE2PointXYVelocity::setDeltaT(const double deltaT)
{
    _deltaT = deltaT;
}

/**
 * EdgeVelocity only
 */
EdgeVelocityOnly::EdgeVelocityOnly() : BaseUnaryEdge<3, Eigen::Vector3d, VertexPointXYZ>()
{
}

void EdgeVelocityOnly::linearizeOplus()
{
    _jacobianOplusXi = Eigen::MatrixXd::Identity(3, 3);
}

void EdgeVelocityOnly::computeError()
{
    const g2o::VertexPointXYZ *v1 = static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);

    _error = v1->estimate() - _measurement;
}

void EdgeVelocityOnly::setMeasurement(const Eigen::Vector3d m)
{
    _measurement = m;
}

bool EdgeVelocityOnly::write(std::ostream &os) const
{
    os << measurement()[0] << " " << measurement()[1] << " " << measurement()[2] << " ";
    os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1) << " " << information()(0, 2) << " " << information()(2, 2) << " " << information()(1, 2);
    return os.good();
}

bool EdgeVelocityOnly::read(std::istream &is)
{
    return true;
}

/**
 * Edge for Velocity and Pose Correlation
*/
// EdgeVelocitySE2::EdgeVelocitySE2() : BaseMultiEdge<3, Eigen::Vector3d>()
// EdgeVelocitySE2::EdgeVelocitySE2() : BaseMultiEdge<2, Eigen::Vector2d>()
// EdgeVelocitySE2::EdgeVelocitySE2() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXYZ>()
// EdgeVelocitySE2::EdgeVelocitySE2() : BaseBinaryEdge<1, double, VertexSE2, VertexPointXYZ>()
EdgeVelocitySE2::EdgeVelocitySE2() : BaseBinaryEdge<3, Eigen::Vector3d, VertexSE2, VertexPointXYZ>()

{
    // resize(3);
    // std::cout <<"EdgeVelocitySE2()" << std::endl;
}

void EdgeVelocitySE2::log(int frame_id)
{
    // const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
    std::string frame_id_str = std::to_string(frame_id);

    // const VertexPointXY *vj = static_cast<const VertexPointXY *>(_vertices[1]);

    std::string log_file = "/home/hong/Desktop/PoseError/EdgeError/" + 
                std::string(6 - frame_id_str.length(), '0').append(frame_id_str) + "_EdgeVelocitySE2.csv";

    csvFile.open(log_file);
    csvFile << "iteration,Ti.x,Ti.y,Ti.theta,Tj.x,Tj.y,Tj.theta,"
            << "Velocity.x,Velocity.y,Velocity.theta,VelocityPrior.x,VelocityPrior.y,VelocityPrior.theta,Error.x,Error.y,Error.theta\n";

    iteration = 0;
}

void EdgeVelocitySE2::computeError()
{
    const VertexPointXYZ *v0 = static_cast<const VertexPointXYZ *>(_vertices[0]);
    // const VertexPointXY *v0 = static_cast<const VertexPointXY *>(_vertices[0]);
    Eigen::Vector3d v0Estimate = v0->estimate();

    // i > j
    const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[1]);
    // const VertexSE2 *vj = static_cast<const VertexSE2 *>(_vertices[2]);

    SE2 rt = vj->estimate().inverse() * vi->estimate();
    Eigen::Vector3d velocityPrior = rt.toVector() / _deltaT;
    // if (iteration == 0)
    // {
    //     std::cout << "velocityPrior: " << velocityPrior << std::endl;
    //     std::cout << " v0->estimate(): " << v0->estimate() << std::endl;
    // }

    _error = v0->estimate() - velocityPrior;

    // Eigen::Vector2d velocityPriorXY = velocityPrior.head<2>(); // Computed from two poses
    // Eigen::Vector2d velocityOptimized;                         // Velocity to be optimized
    // velocityOptimized(0) = v0Estimate(0);
    // velocityOptimized(1) = v0Estimate(1);
    // // _error = velocityOptimized -velocityPriorXY;

    // _error(0,0) = velocityOptimized(0) -velocityPriorXY(0);
    // // _error(0,0) = velocityOptimized(1) -velocityPriorXY(1);

    // Log
    iteration++;
    Vector3 vTi = vi->estimate().toVector();
    Vector3 vTj = vj->estimate().toVector();
    Vector3 velocity = v0->estimate();
    // Vector2 velocity = v0->estimate();

    csvFile << iteration << ","
            << vTi(0) << ","
            << vTi(1) << ","
            << vTi(2) << ","
            << vTj(0) << ","
            << vTj(1) << ","
            << vTj(2) << ","
            << velocity(0) << ","
            << velocity(1) << ","
            << velocity(2) << ","
            // << 0.0 << ","

            << velocityPrior(0) << ","
            << velocityPrior(1) << ","
            << velocityPrior(2) << ","
            << _error(0) << ","
            // << _error(1) << ","
            << 0.0 << ","

            // << _error(2) << "\n";
            << 0.0 << "\n";

    // // Print
    // std::cout << "velocityPrior:" << std::endl
    //           << velocityPrior << std::endl;
    // std::cout << "v0->estimate():" << std::endl
    //           << v0->estimate() << std::endl;
    // std::cout << "velocityPrior _error:" << std::endl
    //           << _error << std::endl
    //           << std::endl;
}

// // ToDo
// void EdgeVelocitySE2::linearizeOplus()
// {
//     Eigen::Matrix<double, 3, 3> eye = Eigen::MatrixXd::Identity(3, 3);

//     // Jocobian of Vi
//     _jacobianOplus[0] = eye;

//     // Jocobian of Ti
//     _jacobianOplus[1] = eye;

//     // Jocobian of Tj
//     _jacobianOplus[2] = eye;
// }

// Set time difference
void EdgeVelocitySE2::setDeltaT(const double deltaT)
{
    _deltaT = deltaT;
}

bool EdgeVelocitySE2::write(std::ostream &os) const
{
    // os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1) << " " << information()(0, 2) << " " << information()(2, 2) << " " << information()(1, 2);
    // os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);
    os << information()(0, 0);

    return os.good();
}

bool EdgeVelocitySE2::read(std::istream &is)
{
    return true;
}

// Register the classes
G2O_REGISTER_TYPE(EDGE_POINT_XY_ONLY, EdgePointXYOnly);
G2O_REGISTER_TYPE(EDGE_VELOCITY_SE2_POINT_FIX, EdgeSE2VelocityPointFixed);
G2O_REGISTER_TYPE(EDGE_VELOCITY_POINT_XY_SE2, EdgeSE2PointXYVelocity);
G2O_REGISTER_TYPE(EDGE_VELOCITY_ONLY, EdgeVelocityOnly);
G2O_REGISTER_TYPE(EDGE_VELOCITY_SE2, EdgeVelocitySE2);

} // namespace g2o