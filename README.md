
# Minimum Curvature Trajectory Planning via Probabilistic Inference for an Autonomous Racing Vehicle

## Prerequisites

For running the PI Racing matlab code you need the following libraries installed on your system:

- CMake
    - Version >= 1.65 (```sudo apt install cmake```)
- Boost (for GTSAM)
    - Version >= 3.0 (```sudo apt install libboost-all-dev```)
- GTSAM (see installation hint below)
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

- There is an example script given called *pi_min_curv.m* (minimum curvature path) 
- Start the script by opening the *src* directory in Matlab and running the *pi_min_curv.m* script to see the optimization for the Berlin 2018 ePrix circuit
- Change the *track_file* variable to "modena_2019.csv" to see the results with comparison for the Modena racetrack
- The script expects the GTSAM matlab library to be in the directory */usr/local/gtsam_toolbox/*

### Citation

```
@article{pi-racing,
  author    = {Salman Bari and
               Ahmad Schoha Haidari and
               Dirk Wollherr},
  title     = {A Fast Approach to Minimum Curvature Raceline Planning via Probabilistic
               Inference},
  journal   = {CoRR},
  volume    = {abs/2203.03224},
  year      = {2022},
  url       = {https://arxiv.org/abs/2203.03224},
  eprinttype = {arXiv},
  eprint    = {2203.03224},
  timestamp = {Thu, 10 Mar 2022 14:39:36 +0100}
}
```
