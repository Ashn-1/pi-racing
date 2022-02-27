
#include <pi_racing/MinimumDistanceFactor.hpp>

namespace pi_racing 
{

    MinimumDistanceFactor::MinimumDistanceFactor(
        const gtsam::SharedNoiseModel& model, gtsam::Key key1, gtsam::Key key2
    ): NoiseModelFactor2<gtsam::Point2, gtsam::Point2>(model, key1, key2) 
    {
    }

    gtsam::Vector MinimumDistanceFactor::evaluateError(
        const gtsam::Point2& p1, 
        const gtsam::Point2& p2, 
        boost::optional<gtsam::Matrix&> H1, 
        boost::optional<gtsam::Matrix&> H2
    ) const 
    {
        if (H1) (*H1) = (gtsam::Matrix(2, 2) << -1.0, 0.0, 0.0, -1.0).finished();
        if (H2) (*H2) = (gtsam::Matrix(2, 2) << 1.0, 0.0, 0.0, 1.0).finished();
        return (gtsam::Vector(2) << p2.x() - p1.x(), p2.y() - p1.y()).finished();
    }
}
