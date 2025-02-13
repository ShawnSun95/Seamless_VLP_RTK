#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct VLP {
    double time;

    Vector3d xyz;
    Vector3d std;
    double RSS[20];
    double RSS_std[20]; // max 20 LEDs
} VLP;

typedef struct obsd {
    uint8_t osat;
    uint8_t LLI[2];
    double L[2];
    double P[2];
    uint16_t snr[2];
} obsd;

typedef struct RTK {
    int64_t time;
    int nx;

    double rb[3];

    int nu, nr;
    double *rs;
    double *e;

    int ns;
    int *sat;
    int *iu, *ir;
    double *freq;
    int refidx[30];

    obsd *obs;
    double *azel;

    int stat;
    double rr[6];
} RTK;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

#endif // TYPES_H
