/*
 * IO detection using RTK and VLP observations
 * Output: 0, outdoor; 1, semi-outdoor; 2, semi-indoor; 3, indoor
 */

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/types.h"

extern int IO_detect(RTK rtk, VLP vlp, int Nled, int N, double P1, double P2)
{
    double sum_RSS = 0;
    for (int i = 0; i < Nled; i++) sum_RSS += vlp.RSS[i];
    if (rtk.nu > N && vlp.RSS[0] + vlp.RSS[1] > P1 && vlp.RSS[0] + vlp.RSS[1] < P2){
        return 1;
    } else if (rtk.nu <= N){
        return 2;
    } else {
        return 0;
    }
}