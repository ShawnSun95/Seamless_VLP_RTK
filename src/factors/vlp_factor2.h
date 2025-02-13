/*
 * This factor is for tightly integrated VLP/INS systems.
 * Created by Xiao Sun.
 */

#ifndef VLP_FACTOR2_H
#define VLP_FACTOR2_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/earth.h"
#include "src/common/rotation.h"
#include "src/common/types.h"

const double clight = 299792458.0;

class VLPFactor2 : public ceres::SizedCostFunction<7, 7, 3, 1> {

public:
    explicit VLPFactor2(VLP vlp, Vector3d lever, int Nled_, vector<double> power, 
        vector<double> M_, vector<double> LED_, double h, bool coord_est_)
        : vlp_(std::move(vlp))
        , lever_(std::move(lever)) {
            Nled = Nled_;
            a = new double[Nled];
            M = new double[Nled];
            LED = new double[Nled*3];
            for (int i = 0; i < Nled; i++){
                a[i] = power[i];
                M[i] = M_[i];
                for (int j = 0; j < 3; j++)
                    LED[i * 3 + j] = LED_[i * 3 + j];
            }
            H = h;
            coord_est = coord_est_;
    }

    void updateVlpState(const VLP &vlp) {
        vlp_ = vlp;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p0{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};
        Vector3d dr{parameters[1][0], parameters[1][1], parameters[1][2]};
        double dir_local = parameters[2][0];

        p0 = p0 + q.toRotationMatrix() * lever_;
        Matrix3d Cln;
        Cln << cos(dir_local), - sin(dir_local), 0, 
                sin(dir_local), cos(dir_local), 0, 
                0, 0, 1;
        Vector3d p = Cln * p0 - dr;
        
        Eigen::Map<Eigen::Matrix<double, 7, 1>> residual(residuals);
        residual.setZero();
        if (jacobians) {
            if (jacobians[0] && jacobians[1] && jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
                Eigen::Map<Eigen::Matrix<double, 7, 3, Eigen::RowMajor>> jacobian1(jacobians[1]);
                Eigen::Map<Eigen::Matrix<double, 7, 1>> jacobian2(jacobians[2]);
                jacobian.setZero();
                jacobian1.setZero();
                jacobian2.setZero();

                Vector3d unit(0, 0, -1);
                Vector3d n_PD = Cln * q.toRotationMatrix() * unit;
                Vector3d n_LED{0, 0, -1};

                double sum_RSS = 0;
                for (int i = 0; i < Nled; i++) sum_RSS += vlp_.RSS[i];
                if(sum_RSS < 0.5 * Nled) {
                // if(vlp_.RSS[1] < 0.5) {
                    // if(vlp_.time > 1703682952 && vlp_.time < 1703682974)
                        // std::cout << vlp_.time - 1703682952 << std::endl;
                    return true;
                }
                
                for (int i = 0; i < Nled; i++){
                    double h = LED[i*3+2] - H;
                    double s = sqrt((LED[i*3+1]-p(0))*(LED[i*3+1]-p(0))+(LED[i*3+0]-p(1))*(LED[i*3+0]-p(1)));
                    Vector3d LOS{LED[i*3+1] - p(0), LED[i*3+0] - p(1), -h}; // NED system
                    double cos1 = LOS.dot(n_PD) / sqrt(h * h + s * s);
                    double cos2 = LOS.dot(n_LED) / sqrt(h * h + s * s);
                    double P = a[i] * cos1 * pow(cos2, M[i]) / (h * h + s * s);
                    residual(i, 0) = P - vlp_.RSS[i];
                    residual(i, 0) = residual(i, 0) / vlp_.RSS_std[i];

                    jacobian(i, 0) = P * (-n_PD[0] / LOS.dot(n_PD) + (3 + M[i])*(LED[i*3+1] - p(0))/(h * h + s * s));
                    jacobian(i, 1) = P * (-n_PD[1] / LOS.dot(n_PD) + (3 + M[i])*(LED[i*3+0] - p(1))/(h * h + s * s));
                    if(coord_est && (i==0 || i==1) && vlp_.RSS[0] + vlp_.RSS[1] > 0.7 * sum_RSS && sum_RSS < 22){ //estimate transfer parameters
                        jacobian1(i, 0) = -jacobian(i, 0);
                        jacobian1(i, 1) = -jacobian(i, 1);
                        jacobian2(i, 0) = (-sin(dir_local)*p0(0) -cos(dir_local)*p0(1)) * jacobian(i, 0) + 
                            (cos(dir_local)*p0(0) -sin(dir_local)*p0(1)) * jacobian(i, 1);
                    }
                    jacobian.block<1, 3>(i, 0) = jacobian.block<1, 3>(i, 0) * Cln;
                    // jacobian(i, 2) = P * (-n_PD[2] / LOS.dot(n_PD) -M[i]*n_LED[2]/n_LED.dot(LOS) + (3 + M[i])*(-LED[i][2] + p(2))/(h * h + s * s));

                    jacobian.block<1, 3>(i, 3) = -P * LOS.cross(n_PD) / LOS.dot(n_PD);
                    jacobian.block<1, 3>(i, 3) = jacobian.block<1, 3>(i, 3) * Cln * q.toRotationMatrix();
                    jacobian.block<1, 3>(i, 3) = jacobian.block<1, 3>(i, 3) + jacobian.block<1, 3>(i, 0) * (-q.toRotationMatrix() * Rotation::skewSymmetric(lever_));
                    jacobian.block<1, 7>(i, 0) = jacobian.block<1, 7>(i, 0) / vlp_.RSS_std[i];
                    jacobian1.block<1, 3>(i, 0) = jacobian1.block<1, 3>(i, 0) / vlp_.RSS_std[i];
                    jacobian2.block<1, 1>(i, 0) = jacobian2.block<1, 1>(i, 0) / vlp_.RSS_std[i];
                }

                residual(6, 0) = (p(2) + H) / 0.0005;//smaller?
                jacobian(6, 2) = 1 / 0.0005;
            }
        }
        return true;
    }

private:
    VLP vlp_;
    Vector3d lever_;

    bool coord_est;
    int Nled;
    double *a;
    double *M;
    double *LED;
    double H;
};

#endif // VLP_FACTOR2_H
