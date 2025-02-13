#ifndef GNSSFILELOADER_H
#define GNSSFILELOADER_H

#include "fileloader.h"
#include "src/common/angle.h"
#include "src/common/types.h"

class GnssFileLoader : public FileLoader {

public:
    GnssFileLoader() = delete;
    explicit GnssFileLoader(const string &filename) {
        fp_bin = fopen(filename.c_str(), "rb");
    }

    const RTK next_bin(double starttime, int GPSweek, double gps0) {
        RTK rtk;

        // read states
        fread(&rtk.time, sizeof(rtk.time), 1, fp_bin);// GPS time
        if(starttime < 1e9){
            rtk.time = rtk.time - GPSweek * 86400 * 7 - gps0; // convert time to weektime
        } else {
            rtk.time = rtk.time - 18; // convert to UTC
        }

        fread(&rtk.nx, sizeof(int), 1, fp_bin);

        // read base station pos
        fread(rtk.rb, sizeof(double), 3, fp_bin);

        // read ephemeris and unit vector
        fread(&rtk.nu, sizeof(int), 1, fp_bin);
        fread(&rtk.nr, sizeof(int), 1, fp_bin);
        rtk.rs = new double[(rtk.nu+rtk.nr)*6];
        rtk.e = new double[(rtk.nu+rtk.nr)*3];
        fread(rtk.rs, sizeof(double), (rtk.nu+rtk.nr)*6, fp_bin);
        fread(rtk.e, sizeof(double), (rtk.nu+rtk.nr)*3, fp_bin);

        // read satellite index
        fread(&rtk.ns, sizeof(int), 1, fp_bin);
        rtk.sat = new int[rtk.ns];
        rtk.iu = new int[rtk.nu];
        rtk.ir = new int[rtk.nr];
        fread(rtk.sat, sizeof(int), rtk.ns, fp_bin);
        fread(rtk.iu, sizeof(int), rtk.nu, fp_bin);
        fread(rtk.ir, sizeof(int), rtk.nr, fp_bin);
        rtk.freq = new double[(rtk.nu+rtk.nr)*2];
        fread(rtk.freq, sizeof(double), (rtk.nu+rtk.nr)*2, fp_bin);
        fread(rtk.refidx, sizeof(int), 30, fp_bin);

        // read observations
        rtk.obs = new obsd[rtk.nu+rtk.nr];
        for (int i=0;i < rtk.nu+rtk.nr; i++){
            fread(&rtk.obs[i].osat, sizeof(uint8_t), 1, fp_bin);
            fread(rtk.obs[i].LLI, sizeof(uint8_t), 2, fp_bin);
            fread(rtk.obs[i].L, sizeof(double), 2, fp_bin);
            fread(rtk.obs[i].P, sizeof(double), 2, fp_bin);
            fread(rtk.obs[i].snr, sizeof(uint16_t), 2, fp_bin);
        }
        rtk.azel = new double[(rtk.nu+rtk.nr)*2];
        fread(rtk.azel, sizeof(double), (rtk.nu+rtk.nr)*2, fp_bin);

        fread(&rtk.stat, sizeof(int), 1, fp_bin);
        fread(rtk.rr, sizeof(double), 6, fp_bin);

        return rtk;
    }

    void free(RTK rtk){
        delete rtk.rs;
        delete rtk.e;
        delete rtk.sat;
        delete rtk.iu;
        delete rtk.ir;
        delete rtk.freq;
        delete rtk.obs;
        delete rtk.azel;
    }

    bool isEOF(){
        bool flag = feof(fp_bin);
        return flag;
    }

private:
    FILE *fp_bin = NULL;
};

#endif // GNSSFILELOADER_H
