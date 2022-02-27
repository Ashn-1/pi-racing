/**
 * @file RacetrackBoundingFactor.h
 * @author Ahmad Haidari
 * @date November 18, 2021
 * @brief Bounds each point to the normal of the centerline point up until the racetrack boundaries.
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace pi_racing 
{

    class RacetrackBoundingFactor: public gtsam::NoiseModelFactor1<gtsam::Point2> 
    {

        private:
            gtsam::Point2 m_leftBoundary;
            gtsam::Point2 m_rightBoundary;
            double m_boundaryDistance;

            gtsam::Vector2 m_heading;
            double m_headingLength;

        public:

            RacetrackBoundingFactor(
                const gtsam::SharedNoiseModel& model, 
                gtsam::Key key1,
                double leftBoundaryX,
                double leftBoundaryY,
                double rightBoundaryX,
                double rightBoundaryY
            );

            gtsam::Vector evaluateError(
                const gtsam::Point2& p1, 
                boost::optional<gtsam::Matrix&> H1 = boost::none
            ) const;
    };
}
