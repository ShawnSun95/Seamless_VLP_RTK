#ifndef IMUFILELOADER_H
#define IMUFILELOADER_H

#include "fileloader.h"
#include "src/common/types.h"

class ImuFileLoader : public FileLoader {

public:
    ImuFileLoader() = delete;
    ImuFileLoader(const string &filename, Matrix3d Cbv_matrix, int rate = 200, int type = 1) {
        Cbv = Cbv_matrix;

        imu_type = type;

        int columns = 7;
        switch(imu_type){
        case 1:
            columns = 7;break;
        case 2:
            columns = 14;break;
        }

        open(filename, columns, FileLoader::TEXT);

        dt_ = 1.0 / (double) rate;

        imu_.time = 0;
    }

    const IMU &next(double gravity) {
        imu_pre_ = imu_;

        data_ = load();

        imu_.time = data_[0];

        if(imu_type == 1){
            memcpy(imu_.dtheta.data(), &data_[4], 3 * sizeof(double));
            memcpy(imu_.dvel.data(), &data_[1], 3 * sizeof(double));

            double dt = imu_.time - imu_pre_.time;
            if (dt < 0.1) {
                imu_.dt = dt;
            } else {
                imu_.dt = dt_;
            }
            // IMU acce: NED, normalized; gyro: arc
            imu_.dtheta = imu_.dtheta * imu_.dt;
            imu_.dvel = imu_.dvel * imu_.dt * gravity;
        } else if(imu_type == 2){
            memcpy(imu_.dtheta.data(), &data_[1], 3 * sizeof(double));
            memcpy(imu_.dvel.data(), &data_[4], 3 * sizeof(double));

            double dt = imu_.time - imu_pre_.time;
            if (dt < 0.1) {
                imu_.dt = dt;
            } else {
                imu_.dt = dt_;
            }
            // IMU: ENU; gyro: arc
            imu_.dtheta = imu_.dtheta * imu_.dt;
            imu_.dvel = imu_.dvel * imu_.dt;

            double temp = imu_.dtheta[0];
            imu_.dtheta[0] = imu_.dtheta[1];
            imu_.dtheta[1] = temp;
            imu_.dtheta[2] = - imu_.dtheta[2];

            temp = imu_.dvel[0];
            imu_.dvel[0] = imu_.dvel[1];
            imu_.dvel[1] = temp;
            imu_.dvel[2] = - imu_.dvel[2];
        }else{
            memcpy(imu_.dtheta.data(), &data_[4], 3 * sizeof(double));
            memcpy(imu_.dvel.data(), &data_[1], 3 * sizeof(double));

            double dt = imu_.time - imu_pre_.time;
            if (dt < 0.1) {
                imu_.dt = dt;
            } else {
                imu_.dt = dt_;
            }
            // IMU: NED, incremental; gyro: degree
            imu_.dtheta = imu_.dtheta / 180 * M_PI;
        }

        imu_.dtheta = Cbv * imu_.dtheta;
        imu_.dvel = Cbv * imu_.dvel;

        return imu_;
    }

private:
    int imu_type;
    double dt_;

    IMU imu_, imu_pre_;
    vector<double> data_;
    Matrix3d Cbv;
};

#endif // IMUFILELOADER_H
