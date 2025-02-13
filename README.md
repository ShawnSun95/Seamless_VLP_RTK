# Seamless_VLP_RTK

## Optimization-Based VLP/RTK/INS Integrated Navigation System

Seamless_VLP_RTK is derived from [OB_GINS_VLP](https://github.com/ShawnSun95/OB_GINS_VLP).

Loosely coupled and tightly coupled integration are both realized. We recommend you to use visual studio code on linux to run our program. We have provided the configuration files in `.vscode/`.

## 1 Prerequisites

### 1.1 System and compiler

We recommend you use Ubuntu 18.04 or Ubuntu 20.04 with the newest compiler (gcc>=8.0).

```shell
# Ubuntu 18.04 or 20.04

# gcc-8
sudo apt install gcc-8 g++-8
```

### 1.2 GTest (needed for new version of abseil-cpp)

```shell
sudo apt-get install libgtest-dev libgmock-dev
```

### 1.3 abseil-cpp

Follow [abseil-cpp installation instructions](https://abseil.io/docs/cpp/quickstart-cmake.html).

Don't forget to `sudo make install` after compiling.

### 1.4 Ceres

Follow [Ceres installation instructions](http://ceres-solver.org/installation.html). 

The version should be lower than 2.2.0. For example, [2.1.0](http://ceres-solver.org/ceres-solver-2.1.0.tar.gz).

### 1.5 yaml-cpp

```shell
sudo apt install libyaml-cpp-dev
```

## 2 Build Seamless_VLP_RTK and run demo

Once the prerequisites have been installed, you can clone this repository and build Seamless_VLP_RTK as follows:

```shell
# Clone the repository
git clone git@github.com:ShawnSun95/Seamless_VLP_RTK.git

# Build
cd Seamless_VLP_RTK
mkdir build && cd build

# gcc
cmake ../ -DCMAKE_BUILD_TYPE=Release

make -j3

# Run demo dataset
cd ../
./bin/seamless ./config/20231227_1_para.yaml

# Wait until the program finish
```

## 3 Plot the results and evaluate the accuracy

We provide a program to plot the navigation results and evaluate the accuracy based on a ground truth positions. You could follow an example:

```shell
python3 ./plot_results.py --optimized_poses ./dataset/20231227_1/output.nav --ground_truth ./dataset/20231227_1/gt_2023-12-27-21-15-08.txt 
```

If you use Seamless_VLP_RTK in an academic work, please cite:

    @article{SUN2025102781,
        title = {Tightly coupled integration of Visible Light Positioning, GNSS, and INS for indoor/outdoor transition areas},
        journal = {Information Fusion},
        volume = {117},
        pages = {102781},
        year = {2025},
        issn = {1566-2535},
        doi = {https://doi.org/10.1016/j.inffus.2024.102781},
        author = {Xiao Sun and Yuan Zhuang and Zhenqi Zheng and Hao Zhang and Binliang Wang and Xuan Wang and Jiasheng Zhou},
    }