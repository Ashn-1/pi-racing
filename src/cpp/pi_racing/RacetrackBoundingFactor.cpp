/**
 * @file RacetrackNormalBoundingFactor.cpp
 * @author Ahmad Haidari
 * @date November 18, 2021
 **/

#include <pi_racing/RacetrackBoundingFactor.hpp>
#include <cmath>
#include <boost/algorithm/clamp.hpp>

namespace pi_racing {
    RacetrackBoundingFactor::RacetrackBoundingFactor(
        const gtsam::SharedNoiseModel& model, 
        gtsam::Key key1,
        double leftBoundaryX,
        double leftBoundaryY,
        double rightBoundaryX,
        double rightBoundaryY
    ) : gtsam::NoiseModelFactor1<gtsam::Point2>(model, key1), 
        m_leftBoundary(gtsam::Point2(leftBoundaryX, leftBoundaryY)),
        m_rightBoundary(gtsam::Point2(rightBoundaryX, rightBoundaryY)),
        m_boundaryDistance(std::sqrt((m_rightBoundary - m_leftBoundary).squaredNorm())){
            m_heading = m_rightBoundary - m_leftBoundary;
            m_headingLength = m_heading.norm();
            m_heading.normalize();
    }

    gtsam::Vector RacetrackBoundingFactor::evaluateError(
        const gtsam::Point2& p1, 
        boost::optional<gtsam::Matrix&> H1
    ) const {
        // Following algorithm from https://stackoverflow.com/questions/51905268/how-to-find-closest-point-on-line#51906100

        // Project from point to line, but clamp the result in order to be using a finite line
        double length = (p1 - m_leftBoundary).dot(m_heading);
        // Clamping
        bool isOutOfBounds = (length < 0.0 || length > m_headingLength);
        length = boost::algorithm::clamp(length, 0.0, m_headingLength);

        // Compute nearest point on the finite line
        gtsam::Vector2 pointOnLine = m_leftBoundary + m_heading * length;

        // Compute the Jacobians depending on the case (inbounds or outbounds)
        if (H1) {
            if (isOutOfBounds) {
                (*H1) = (gtsam::Matrix(2, 2) << -1.0, 0.0, 0.0, -1.0).finished();
            } else {
                double x_heading_sq = m_heading.x() * m_heading.x();
                double y_heading_sq = m_heading.y() * m_heading.y();
                double xy_heading = m_heading.x() * m_heading.y();

                (*H1) = (gtsam::Matrix(2, 2) << 
                    x_heading_sq - 1, xy_heading,
                    xy_heading, y_heading_sq - 1
                ).finished();
            }
        }        

        gtsam::Vector error = gtsam::Vector2(
            pointOnLine.x() - p1.x(), 
            pointOnLine.y() - p1.y()
        );

        return error;
    }
}
