#ifndef VLP_FACTOR_H
#define VLP_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/rotation.h"
#include "src/common/types.h"

class VLPFactor : public ceres::SizedCostFunction<3, 7> {

public:
    explicit VLPFactor(VLP vlp, Vector3d lever)
        : vlp_(std::move(vlp))
        , lever_(std::move(lever)) {
    }

    void updateVlpState(const VLP &vlp) {
        vlp_ = vlp;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);

        error = p + q.toRotationMatrix() * lever_ - vlp_.xyz;

        Matrix3d weight = Matrix3d::Zero();
        weight(0, 0)    = 1.0 / vlp_.std[0];
        weight(1, 1)    = 1.0 / vlp_.std[1];
        weight(2, 2)    = 1.0 / vlp_.std[2];

        error = weight * error;

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                jacobian_pose.setZero();

                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
                jacobian_pose.block<3, 3>(0, 3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);

                jacobian_pose = weight * jacobian_pose;
            }
        }

        return true;
    }

private:
    VLP vlp_;
    Vector3d lever_;
};

#endif // VLP_FACTOR_H
