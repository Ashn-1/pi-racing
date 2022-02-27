
# Minimum Curvature Trajectory Planning via Probabilistic Inference for an Autonomous Racing Vehicle

## Prerequisites

For running the PI Racing matlab code you need the following libraries installed on your system:

- GTSAM (installed from branch 'wrap-export')
- PI Racing Factors (from this repo)

The code was written and tested on Ubuntu 18.04.6 LTS and Matlab R2021b.

### GTSAM

Clone the GTSAM repo from [Github](https://github.com/borglab/gtsam.git) and change to its root directory, then:

```bash
git checkout wrap-export
mkdir build
cd build
cmake ..
sudo make install
```

### PI Racing Factors

Clone this project and change to its root directory, then:

```bash
cd src/cpp
mkdir build && cd build
cmake ..
sudo make install
```

### Matlab

Open the *src* directory in Matlab and run the *pi_min_curv.m* to see the optimization for the Berlin 2018 ePrix circuit. The script expects the GTSAM matlab library to be in the directory */usr/local/gtsam_toolbox/*
