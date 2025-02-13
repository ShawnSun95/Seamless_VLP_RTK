/*
 * This factor is for tightly integrated VLP/RTK/INS systems.
 * Created by Xiao Sun.
 */

#ifndef GNSS_FACTOR_H
#define GNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/earth.h"
#include "src/common/rotation.h"
#include "src/common/types.h"

class GNSSFactor : public ceres::CostFunction {

public:
    GNSSFactor() = delete;

    explicit GNSSFactor(RTK rtk_, Vector3d arm, Vector3d station_origin): 
        rtk(rtk_), leverarm(arm), origin(station_origin) {

        // parameter
        *mutable_parameter_block_sizes() = {7};

        // residual
        set_num_residuals(3); 
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
        residual.setZero();
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
                jacobian.setZero();
                
                if(rtk.stat == 1 || rtk.stat == 2){ //fix or float
                    Vector3d r_update(rtk.rr[0], rtk.rr[1], rtk.rr[2]);
                    Vector3d ecef0 = Earth::blh2ecef(origin);
                    Matrix3d cn0e  = Earth::cne(origin);
                    Vector3d rr = ecef0 + cn0e * (p + q.toRotationMatrix() * leverarm);

                    residual.block<3, 1>(0, 0) = rr - r_update;
                    jacobian.block<3, 3>(0, 0) = cn0e;
                    jacobian.block<3, 3>(0, 3) = -cn0e * q.toRotationMatrix() * Rotation::skewSymmetric(leverarm);

                    if(rtk.stat == 1){
                        residual.block<3, 1>(0, 0) = residual.block<3, 1>(0, 0) / 0.02;
                        jacobian.block<3, 7>(0, 0) = jacobian.block<3, 7>(0, 0) / 0.02;
                    } else if(rtk.time < 1703682952){
                        residual.block<3, 1>(0, 0) = residual.block<3, 1>(0, 0) / 0.1;
                        jacobian.block<3, 7>(0, 0) = jacobian.block<3, 7>(0, 0) / 0.1;
                    }
                }
            }
        }
        return true;
    }

private:
    RTK rtk;
    Vector3d leverarm;
    Vector3d origin;
};

#endif // GNSS_FACTOR_H
