/**
 * @file MinimumDistanceFactor.h
 * @author Ahmad Haidari
 * @date November 02, 2021
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace pi_racing {

    class MinimumDistanceFactor: public gtsam::NoiseModelFactor2<gtsam::Point2, gtsam::Point2> {

        public:
            MinimumDistanceFactor(
                const gtsam::SharedNoiseModel& model, gtsam::Key key1, gtsam::Key key2
            );

            gtsam::Vector evaluateError(
                const gtsam::Point2& p1, 
                const gtsam::Point2& p2, 
                boost::optional<gtsam::Matrix&> H1 = boost::none, 
                boost::optional<gtsam::Matrix&> H2 = boost::none
            ) const;
    };
}
