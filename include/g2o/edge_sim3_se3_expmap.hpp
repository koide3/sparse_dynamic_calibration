#ifndef SIM3_ESTIMATION_EDGE_SIM3_SE3_EXPMAP_HPP
#define SIM3_ESTIMATION_EDGE_SIM3_SE3_EXPMAP_HPP

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace g2o {

class EdgeSim3SE3 : public BaseBinaryEdge<6, SE3Quat, VertexSim3Expmap, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSim3SE3() {}
    ~EdgeSim3SE3() override {}

    bool read(std::istream &is) override;
    bool write(std::ostream &os) const override;

    void computeError() override {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

        SE3Quat C(_measurement);
        SE3Quat v1_estimate(v1->estimate().rotation(), v1->estimate().translation() * v1->estimate().scale());
        SE3Quat v2_estimate = v2->estimate();

        SE3Quat error_ = C * v1_estimate * v2->estimate().inverse();

        _error = error_.log();
    }

    void init_v2() {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        VertexSE3Expmap* v2 = static_cast<VertexSE3Expmap*>(_vertices[1]);

        SE3Quat C(_measurement);
        SE3Quat v1_estimate(v1->estimate().rotation(), v1->estimate().translation() * v1->estimate().scale());
        SE3Quat init_guess = C * v1_estimate;

        v2->setEstimate(init_guess);
    }
};

}

#endif
