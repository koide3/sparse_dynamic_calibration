#ifndef KKL_G2O_EDGE_SE3_PLANE_HPP
#define KKL_G2O_EDGE_SE3_PLANE_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace g2o {
class EdgeSE3Plane : public g2o::BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3Expmap, g2o::VertexPlane> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3Plane()
        : BaseBinaryEdge<3, g2o::Plane3D, g2o::VertexSE3Expmap, g2o::VertexPlane>()
    {}
    ~EdgeSE3Plane() override {}

    void computeError() override;

    void setMeasurement(const g2o::Plane3D& m) override {
        _measurement = m;
    }

    virtual bool read(std::istream& is) override;
    virtual bool write(std::ostream& os) const override;
};
}

#endif
