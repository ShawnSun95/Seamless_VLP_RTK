/*
 * This factor is for tightly integrated VLP/INS systems.
 * Created by Xiao Sun.
 */

#ifndef NHC_FACTOR
#define NHC_FACTOR

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/earth.h"
#include "src/common/rotation.h"
#include "src/common/types.h"

class NHCFactor : public ceres::SizedCostFunction<2, 7, 9> {

public:
    explicit NHCFactor(Vector3d lever)
        : lever_(std::move(lever)) {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d v{parameters[1][0], parameters[1][1], parameters[1][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        
        Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals);
        residual.setZero();
        Matrix3d R_wb = q.toRotationMatrix().transpose();
        Vector3d v_b = R_wb * v;
        residual(0, 0) = v_b(1);
        residual(1, 0) = v_b(2);
        residual = residual * 20;
        
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian1(jacobians[0]);
                jacobian1.setZero();
                jacobian1(0, 6) = -2 * q.z() * v(0) + 2 * q.x() * v(2);
                jacobian1(0, 3) = 2 * q.y() * v(0) -4 * q.x() * v(1) + 2 * q.w() * v(2);
                jacobian1(0, 4) = 2 * q.x() * v(0) + 2 * q.z() * v(2);
                jacobian1(0, 5) = -2 * q.w() * v(0) -4 * q.z() * v(1) + 2 * q.y() * v(2);
                jacobian1(1, 6) = 2 * q.y() * v(0) - 2 * q.x() * v(1);
                jacobian1(1, 3) = 2 * q.z() * v(0) - 2 * q.w() * v(1) -4 * q.x() * v(2);
                jacobian1(1, 4) = 2 * q.w() * v(0) + 2 * q.z() * v(1) -4 * q.y() * v(2);
                jacobian1(1, 5) = 2 * q.x() * v(0) + 2 * q.y() * v(1);
                jacobian1 = jacobian1 * 20;
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> jacobian2(jacobians[1]);
                jacobian2.setZero();
                jacobian2.block<2, 3>(0, 0) = R_wb.block<2, 3>(1, 0);
                jacobian2 = jacobian2 * 20;
            }
        }
        return true;
    }

private:
    Vector3d lever_;
};

#endif // NHC_FACTOR
