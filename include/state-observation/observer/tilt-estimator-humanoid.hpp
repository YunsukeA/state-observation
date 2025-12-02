/**
 * \file      tilt-estimator.hpp
 * \author    Rafael Cisneros, Mehdi Benallegue
 * \date       2018
 * \brief      Version of the Tilt Estimator that implements all the necessary functions to perform the estimation for
 * humanoid robots.
 *
 * \details
 *
 *
 */

#ifndef TILTESTIMATORHUMANOIDHPP
#define TILTESTIMATORHUMANOIDHPP

#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <state-observation/api.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <state-observation/observer/zero-delay-observer.hpp>

namespace stateObservation
{

/**
 * \class  TiltEstimatorHumanoid
 * \brief  Version of the Tilt Estimator for humanoid robots.
 *
 */
class STATE_OBSERVATION_DLLAPI TiltEstimatorHumanoid : public TiltEstimator
{
public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li gamma : parameter related to the orthogonality
  ///  \li dt : sampling time
  TiltEstimatorHumanoid(double alpha, double beta, double gamma, double dt);

  /// sets the position of the IMU sensor in the control frame
  void setSensorPositionInC(const Vector3 & p)
  {
    p_S_C_ = p;
  }

  Vector3 getSensorPositionInC()
  {
    return p_S_C_;
  }

  /// sets the oriantation of the IMU sensor in the control frame
  void setSensorOrientationInC(const Matrix3 & R)
  {
    R_S_C_ = R;
  }
  Matrix3 getSensorOrientationInC()
  {
    return R_S_C_;
  }

  Vector3 getVirtualLocalVelocityMeasurement()
  {
    return x1_;
  }

  /// sets teh linear velocity of the IMU sensor in the control frame
  void setSensorLinearVelocityInC(const Vector3 & v)
  {
    v_S_C_ = v;
  }

  Vector3 getSensorLinearVelocityInC()
  {
    return v_S_C_;
  }

  /// sets the angular velocity of the IMU sensor in the control frame
  void setSensorAngularVelocityInC(const Vector3 & w)
  {
    w_S_C_ = w;
  }
  Vector3 getSensorAngularVelocityInC()
  {
    return w_S_C_;
  }

  /// sets the velocity of the control origin in the world frame
  /// this velocity has to be expressed in the control frame.
  void setControlOriginVelocityInW(const Vector3 & v)
  {
    v_C_ = v;
  }
  Vector3 getControlOriginVelocityInW()
  {
    return v_C_;
  }

  /// @brief informs the estimator that x1hat (the estimate of the local linear velocity of the IMU in the world) needs
  /// to be reset.
  /// @copydetails checkResetX1hat()
  void resetImuLocVelHat();

/// prevent c++ overloaded virtual function warning
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Woverloaded-virtual"
#else
#  if defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Woverloaded-virtual"
#  endif
#endif

  // we also want to use the function setMeasurement from the TiltEstimator class, that is hidden by the following
  using TiltEstimator::setMeasurement;
  /// @brief sets the measurement (accelero and gyro stacked in one vector)
  /// @details the local linear velocity of the IMU in the world is obtained from \p imuControlPos, \p imuControlLinVel
  /// and \p yg_k. The control frame is a frame whose velocity is considered zero in the world frame.
  /// @param imuControlPos position of the IMU frame in the control frame
  /// @param imuControlLinVel linear velocity of the IMU frame in the control frame
  /// @param ya_k accelerometer measurement
  /// @param yg_k gyrometer measurement
  void setMeasurement(const Vector3 & imuControlPos,
                      const Vector3 & imuControlLinVel,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      TimeIndex k);

#if defined(__clang__)
#  pragma clang diagnostic pop
#else
#  if defined(__GNUC__)
#    pragma GCC diagnostic pop
#  endif
#endif
};

} // namespace stateObservation

#endif // TILTESTIMATORHUMANOIDHPP
