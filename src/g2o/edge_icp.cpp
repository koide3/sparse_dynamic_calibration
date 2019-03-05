#include <g2o/edge_icp.hpp>


#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>


namespace g2o {

G2O_REGISTER_TYPE(EDGE_ICP, EdgeICP);

// I/O functions
bool EdgeICP::read(std::istream& is) { return true; }
bool EdgeICP::write(std::ostream& os) const { return true; }

// return the error estimate as a 3-vector
void EdgeICP::computeError()
{
    // from <ViewPoint> to <Point>
    const VertexSE3Expmap *vp0 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap *vp1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

    // get vp1 point into vp0 frame
    g2o::SE3Quat se3 = vp0->estimate() * vp1->estimate().inverse();
    Eigen::Vector3d p1 = se3 * measurement().pos1;

    Eigen::Vector3d diff = _error = p1 - measurement().pos0;
    _error = diff;

    if(point_to_plane) {
        Eigen::Vector3d n1 = se3.rotation() * measurement().normal1;
        _error = diff.array() * measurement().normal0.array() + diff.array() * n1.array();
    }
}

}
