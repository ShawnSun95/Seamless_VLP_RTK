#ifndef IMU_ERROR_FACTOR_H
#define IMU_ERROR_FACTOR_H

#include "preintegration_base.h"

#include <ceres/ceres.h>

class ImuErrorFactor : public ceres::CostFunction {

public:
    explicit ImuErrorFactor(std::shared_ptr<PreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {

        *mutable_parameter_block_sizes() = preintegration_->imuErrorNumBlocksParameters();
        set_num_residuals(preintegration_->imuErrorNumResiduals());
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {

        // parameters: vel[3], bg[3], ba[3]

        preintegration_->imuErrorEvaluate(parameters, residuals);

        if (jacobians) {
            if (jacobians[0]) {
                preintegration_->imuErrorJacobian(jacobians[0]);
            }
        }

        return true;
    }

private:
    std::shared_ptr<PreintegrationBase> preintegration_;
};

#endif // IMU_ERROR_FACTOR_H
