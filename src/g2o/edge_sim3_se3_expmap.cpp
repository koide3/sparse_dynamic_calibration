#include <g2o/edge_sim3_se3_expmap.hpp>

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace g2o {

G2O_REGISTER_TYPE(EDGE_SIM3_SE3, EdgeSim3SE3);


bool EdgeSim3SE3::read(std::istream &is) {
    Eigen::Matrix<double, 7, 1> meas;
    for(int i=0; i<7; i++) {
        is >> meas[i];
    }

    setMeasurement(SE3Quat(meas));

    for(int i=0; i<6; i++) {
        for(int j=i; j<6; j++) {
            is >> information()(i, j);
            if(i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }

    return true;
}

bool EdgeSim3SE3::write(std::ostream &os) const {
    SE3Quat delta(measurement());
    for(int i=0; i<7; i++) {
        os << delta[i] << " ";
    }

    for(int i=0; i<6; i++) {
        for(int j=i; j<6; j++) {
            os << " " << information()(i, j);
        }
    }

    return os.good();
}


}
