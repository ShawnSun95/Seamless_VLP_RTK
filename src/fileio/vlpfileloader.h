#ifndef VLPFILELOADER_H
#define VLPFILELOADER_H

#include "fileloader.h"
#include "src/common/angle.h"
#include "src/common/types.h"

class VLPFileLoader : public FileLoader {

public:
    VLPFileLoader() = delete;
    explicit VLPFileLoader(const string &filename, int Nled, std::vector<double>err) {
        open(filename, 4 + Nled * 2, FileLoader::TEXT);
        N = Nled;
        for (int i=0; i<3; i++)
            vlp_.std(i)=err[i];
    }

    const VLP &next() {
        data_ = load();
        
        vlp_.time = data_[0];
        
        // NED
        memcpy(vlp_.xyz.data(), &data_[1], 3 * sizeof(double));
        vlp_.xyz(2) = -vlp_.xyz(2);

        if(data_.size() == 4 + N){
            for (int i = 0; i < N; i++){
                vlp_.RSS[i] = data_[i + 4];
                vlp_.RSS_std[i] = 0.1;
            }
        } else if(data_.size() == 4 + N * 2){
            for (int i = 0; i < N; i++){
                vlp_.RSS[i] = data_[i + 4];
                vlp_.RSS_std[i] = data_[i + N + 4];
            }
        }
        return vlp_;
    }

    bool isEOF(){
        bool flag;
        if(scheme_ == 1) {
            flag = filefp_.eof();
        } else {
            flag = feof(fp_bin);
        }
        return flag;
    }

private:
    FILE *fp_bin = NULL;
    int scheme_ = 1;
    VLP vlp_;
    int N;
    vector<double> data_;
};

#endif // VLPFILELOADER_H
