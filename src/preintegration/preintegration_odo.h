#ifndef PREINTEGRATION_ODO_H
#define PREINTEGRATION_ODO_H

#include "preintegration_base.h"

class PreintegrationOdo : public PreintegrationBase {

public:
    PreintegrationOdo(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0, IntegrationState state);

    Eigen::MatrixXd evaluate(const IntegrationState &state0, const IntegrationState &state1,
                             double *residuals) override;

    Eigen::MatrixXd residualJacobianPose0(const IntegrationState &state0, const IntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianPose1(const IntegrationState &state0, const IntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix0(const IntegrationState &state0, const IntegrationState &state1,
                                         double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix1(const IntegrationState &state0, const IntegrationState &state1,
                                         double *jacobian) override;
    int numResiduals() override;
    vector<int> numBlocksParameters() override;

    static IntegrationStateData stateToData(const IntegrationState &state);
    static IntegrationState stateFromData(const IntegrationStateData &data);
    void constructState(const double *const *parameters, IntegrationState &state0, IntegrationState &state1) override;

    int imuErrorNumResiduals() override;
    vector<int> imuErrorNumBlocksParameters() override;
    void imuErrorEvaluate(const double *const *parameters, double *residuals) override;
    void imuErrorJacobian(double *jacobian) override;

protected:
    void integrationProcess(unsigned long index) override;
    void resetState(const IntegrationState &state) override;

    void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) override;

private:
    void resetState(const IntegrationState &state, int num);
    void setNoiseMatrix();

public:
    static constexpr int NUM_MIX = 10;

private:
    static constexpr int NUM_STATE = 19;
    static constexpr int NUM_NOISE = 16;

    static constexpr int NUM_ERROR_RESIDUAL = 7;

    Matrix3d cvb_;
    Vector3d lodo_;

    Vector3d corrected_s_;
};

#endif // PREINTEGRATION_ODO_H
