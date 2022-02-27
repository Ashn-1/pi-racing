/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     example.i
 * @brief    Example wrapper interface file
 * @author   Richard Roberts, Varun Agrawal
 */

// This is an interface file for automatic MATLAB wrapper generation.
// See `wrap` for full documentation and more examples.

#include <pi_racing/MinimumDistanceFactor.hpp>
#include <pi_racing/MinimumSteeringAngleFactor.hpp>
#include <pi_racing/RacetrackBoundingFactor.hpp>

namespace pi_racing {

/* MINIMUM DISTANCE FACTOR */
virtual class MinimumDistanceFactor : gtsam::NoiseModelFactor {
  MinimumDistanceFactor(
      const gtsam::noiseModel::Base* model, 
      size_t key1, 
      size_t key2
  );

  gtsam::Vector evaluateError(
      const gtsam::Vector& p1, 
      const gtsam::Vector& p2
  ) const;
};

/* MINIMUM STEERING ANGLE FACTOR */
virtual class MinimumSteeringAngleFactor : gtsam::NoiseModelFactor {
  MinimumSteeringAngleFactor(
      const gtsam::noiseModel::Base* model, 
      size_t key1, 
      size_t key2, 
      size_t key3
  );

  gtsam::Vector evaluateError(
      const gtsam::Vector& p1, 
      const gtsam::Vector& p2,
      const gtsam::Vector& p3
  ) const;
};

/* RACETRACK BOUNDING FACTOR */
virtual class RacetrackBoundingFactor : gtsam::NoiseModelFactor {
  RacetrackBoundingFactor(
      const gtsam::noiseModel::Base* model, 
      size_t key1,
      double leftBoundaryX,
      double leftBoundaryY,
      double rightBoundaryX,
      double rightBoundaryY
  );

  gtsam::Vector evaluateError(
      const gtsam::Vector& p1
  ) const;
};

}  // namespace example
