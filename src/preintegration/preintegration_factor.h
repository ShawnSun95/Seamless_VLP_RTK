#ifndef PREINTEGRATION_FACTOR_H
#define PREINTEGRATION_FACTOR_H

#include "preintegration_base.h"

#include <ceres/ceres.h>

class PreintegrationFactor : public ceres::CostFunction {

public:
    PreintegrationFactor() = delete;

    explicit PreintegrationFactor(std::shared_ptr<PreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {

        // parameter
        *mutable_parameter_block_sizes() = preintegration_->numBlocksParameters();

        // residual
        set_num_residuals(preintegration_->numResiduals());
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // construct state
        IntegrationState state0, state1;
        preintegration_->constructState(parameters, state0, state1);

        // residual
        preintegration_->evaluate(state0, state1, residuals);

        if (jacobians) {
            if (jacobians[0]) {
                preintegration_->residualJacobianPose0(state0, state1, jacobians[0]);
            }
            if (jacobians[1]) {
                preintegration_->residualJacobianMix0(state0, state1, jacobians[1]);
            }
            if (jacobians[2]) {
                preintegration_->residualJacobianPose1(state0, state1, jacobians[2]);
            }
            if (jacobians[3]) {
                preintegration_->residualJacobianMix1(state0, state1, jacobians[3]);
            }
        }

        return true;
    }

private:
    std::shared_ptr<PreintegrationBase> preintegration_;
};

#endif // PREINTEGRATION_FACTOR_H
