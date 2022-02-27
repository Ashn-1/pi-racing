
#include <pi_racing/MinimumSteeringAngleFactor.hpp>
#include <cmath>

namespace pi_racing 
{

    MinimumSteeringAngleFactor::MinimumSteeringAngleFactor(
        const gtsam::SharedNoiseModel& model, 
        gtsam::Key key1, 
        gtsam::Key key2,
        gtsam::Key key3
    ): NoiseModelFactor3<gtsam::Point2, gtsam::Point2, gtsam::Point2>(model, key1, key2, key3)
    {
    }

    gtsam::Vector MinimumSteeringAngleFactor::evaluateError(
        const gtsam::Point2& p1, 
        const gtsam::Point2& p2, 
        const gtsam::Point2& p3, 
        boost::optional<gtsam::Matrix&> H1, 
        boost::optional<gtsam::Matrix&> H2,
        boost::optional<gtsam::Matrix&> H3
    ) const 
    {
        if (H1) (*H1) = (gtsam::Matrix(2, 2) << -1.0, 0.0, 0.0, -1.0).finished();
        if (H2) (*H2) = (gtsam::Matrix(2, 2) << 2.0, 0.0, 0.0, 2.0).finished();
        if (H3) (*H3) = (gtsam::Matrix(2, 2) << -1.0, 0.0, 0.0, -1.0).finished();

        double x = (2 * p2.x() - p1.x() - p3.x());
        double y = (2 * p2.y() - p1.y() - p3.y());

        gtsam::Vector2 err = gtsam::Vector2(x, y);

        return err;
    }
}
