/*
 * This factor is for tightly integrated VLP/RTK/INS systems.
 * Created by Xiao Sun.
 */

#ifndef TDGNSS_FACTOR_H
#define TDGNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/earth.h"
#include "src/common/rotation.h"
#include "src/common/types.h"

void find_ij(RTK rtk, int num, int ind[2])
{
    for (int j = 0; j < rtk.nu; j++){
        if(rtk.obs[j].osat == num){
            ind[0] = j;
        } 
    }
    for (int j = rtk.nu; j < rtk.nr + rtk.nu; j++){
        if(rtk.obs[j].osat == num){
            ind[1] = j;
        } 
    }
}

class TDGNSSFactor : public ceres::CostFunction {

public:
    TDGNSSFactor() = delete;

    explicit TDGNSSFactor(RTK rtk_former, RTK rtk_, Vector3d arm, std::vector<double> satpair, 
        Vector3d station_origin): rtk0(rtk_former), 
        rtk(rtk_), leverarm(arm), pair(satpair), origin(station_origin) {

        // int numpair = (int) pair.size()/2;
        // parameter
        *mutable_parameter_block_sizes() = {7, 7};

        // residual
        set_num_residuals(8); // temperarily 5 satpair 
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p0{parameters[0][0], parameters[0][1], parameters[0][2]};
        Vector3d p{parameters[1][0], parameters[1][1], parameters[1][2]};
        Quaterniond q0{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        Quaterniond q{parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]};

        Eigen::Map<Eigen::Matrix<double, 8, 1>> residual(residuals);
        residual.setZero();
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 8, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
                Eigen::Map<Eigen::Matrix<double, 8, 7, Eigen::RowMajor>> jacobian1(jacobians[1]);
                jacobian.setZero();
                jacobian1.setZero();
                
                // use rtk or rtk0?
                //  B=[B;eri-erj];
                //  l=[l;dot(eri-ebi,dxi)-dot(erj-ebj,dxj)-d3r_];
                for (int i = 0; i < 5; i++){  // wrong on first satellite
                    // find the sat index in observations
                    int I[2] = {0}, J[2] = {0}, I0[2] = {0}, J0[2] = {0};

                    find_ij(rtk, pair[i * 2], I);
                    find_ij(rtk, pair[i * 2 + 1], J);
                    find_ij(rtk0, pair[i * 2], I0);
                    find_ij(rtk0, pair[i * 2 + 1], J0);
                    
                    if(I[0]!=0 && I[1]!=0 && J[0]!=0 && J[1]!=0 &&
                        I0[0]!=0 && I0[1]!=0 && J0[0]!=0 && J0[1]!=0){ //only the first band
                        double *obs_r_i = rtk.obs[I[0]].L; // -1?
                        double *obs_b_i = rtk.obs[I[1]].L;
                        double *obs_r_j = rtk.obs[J[0]].L;
                        double *obs_b_j = rtk.obs[J[1]].L;
                        double *obs_r_i0 = rtk0.obs[I0[0]].L;
                        double *obs_b_i0 = rtk0.obs[I0[1]].L;
                        double *obs_r_j0 = rtk0.obs[J0[0]].L;
                        double *obs_b_j0 = rtk0.obs[J0[1]].L;

                        double ddL = obs_r_i[0] - obs_b_i[0] - obs_r_j[0] + obs_b_j[0];
                        double ddL0 = obs_r_i0[0] - obs_b_i0[0] - obs_r_j0[0] + obs_b_j0[0];
                        double f = 0;
                        if(rtk.freq[J[0]*2]!=0) // -1?
                            f = rtk.freq[J[0]*2];
                        else if(rtk.freq[I[0]*2]!=0)
                            f = rtk.freq[I[0]*2];
                        ddL = ddL * clight / f;
                        ddL0 = ddL0 * clight / f;

                        Vector3d rsi(rtk.rs[I[0]*6],rtk.rs[I[0]*6+1],rtk.rs[I[0]*6+2]);
                        Vector3d rsj(rtk.rs[J[0]*6],rtk.rs[J[0]*6+1],rtk.rs[J[0]*6+2]);
                        Vector3d rsi0(rtk0.rs[I0[0]*6],rtk0.rs[I0[0]*6+1],rtk0.rs[I0[0]*6+2]);
                        Vector3d rsj0(rtk0.rs[J0[0]*6],rtk0.rs[J0[0]*6+1],rtk0.rs[J0[0]*6+2]);
                        Vector3d ecef0 = Earth::blh2ecef(origin);
                        Matrix3d cn0e  = Earth::cne(origin);
                        Vector3d rr = ecef0 + cn0e * (p + q.toRotationMatrix() * leverarm);
                        Vector3d rr0 = ecef0 + cn0e * (p0 + q0.toRotationMatrix() * leverarm);
                        Vector3d rb(rtk.rb[0],rtk.rb[1],rtk.rb[2]);

                        Vector3d dxi=rsi-rsi0;
                        Vector3d dxj=rsj-rsj0;
                        Vector3d eri=(rsi-rr)/(rsi-rr).norm();
                        Vector3d ebi=(rsi-rb)/(rsi-rb).norm();
                        Vector3d erj=(rsj-rr)/(rsj-rr).norm();
                        Vector3d ebj=(rsj-rb)/(rsj-rb).norm();

                        double l=dxi.dot(eri-ebi)-dxj.dot(erj-ebj)-(ddL-ddL0)/(rtk.time-rtk0.time);
                        if(abs(l) > (eri-erj).norm() * 2 || abs(ddL-ddL0) < 1e-3) {
                            int debug=1;
                            continue; // outlier
                        }
                        residual(i, 0) = ((eri-erj).dot(rr-rr0)-l) / 0.002;
                        jacobian.block<1, 3>(i, 0) = -(eri-erj).transpose() * cn0e;
                        jacobian1.block<1, 3>(i, 0) = (eri-erj).transpose() * cn0e;
                        jacobian.block<1, 3>(i, 3) = (eri-erj).transpose() * cn0e * q0.toRotationMatrix() * Rotation::skewSymmetric(leverarm);
                        jacobian1.block<1, 3>(i, 3) = -(eri-erj).transpose() * cn0e * q.toRotationMatrix() * Rotation::skewSymmetric(leverarm);
                        jacobian.block<1, 7>(i, 0) = jacobian.block<1, 7>(i, 0)/ 0.002;
                        jacobian1.block<1, 7>(i, 0) = jacobian1.block<1, 7>(i, 0)/ 0.002;
                    }else{ // no common satellite
                        residual(i, 0) = 0;
                    }
                }
            }
        }
        return true;
    }

private:
    RTK rtk, rtk0;
    Vector3d leverarm;
    std::vector<double> pair;
    Vector3d origin;
};

#endif // TDGNSS_FACTOR_H
