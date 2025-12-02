#include <state-observation/observer/tilt-estimator-humanoid.hpp>

namespace stateObservation
{

TiltEstimatorHumanoid::TiltEstimatorHumanoid(double alpha, double beta, double gamma, double dt)
: TiltEstimator(alpha, beta, gamma, dt), p_S_C_(Vector3::Zero()), R_S_C_(Matrix3::Identity()), v_S_C_(Vector3::Zero()),
  w_S_C_(Vector3::Zero()), v_C_(Vector3::Zero())
{
}

void TiltEstimatorHumanoid::setMeasurement(const Vector3 & imuControlPos,
                                           const Vector3 & imuControlLinVel,
                                           const Vector3 & ya_k,
                                           const Vector3 & yg_k,
                                           TimeIndex k)
{
  x1_ = -yg_k.cross(imuControlPos) - imuControlLinVel;

  TiltEstimator::setMeasurement(x1_, ya_k, yg_k, k);
}

void TiltEstimatorHumanoid::resetImuLocVelHat()
{
  x_().segment<3>(0) = x1_;
}

} // namespace stateObservation
