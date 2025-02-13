#ifndef POSE_PARAMETERIZATION_H
#define POSE_PARAMETERIZATION_H

#include "src/common/rotation.h"

class PoseParameterization : public ceres::LocalParameterization {
    // 四元数定义顺序为, x, y, z, w
    // Quaternion order (x, y, z, w)

public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
        Eigen::Map<const Eigen::Vector3d> _p(x);
        Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);

        Eigen::Quaterniond dq = Rotation::rotvec2quaternion(Eigen::Map<const Eigen::Vector3d>(delta + 3));

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

        p = _p + dp;
        q = (_q * dq).normalized();

        return true;
    }

    bool ComputeJacobian(const double *x, double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }

    int GlobalSize() const override {
        return 7;
    }

    int LocalSize() const override {
        return 6;
    }
};

#endif // POSE_PARAMETERIZATION_H
