#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // create a 4D state vector
  x_ = Eigen::VectorXd(4);

  // TODO: Initialize the state covariance matrix P
  P_ = Eigen::MatrixXd(4, 4);
  P_ << 1., 0., 0., 0.,
      0., 1., 0., 0.,
      0., 0., 9999., 0.,
      0., 0., 0., 9999.;

  // measurement covariance
  R_ = Eigen::MatrixXd(2, 2);
  R_ << 0.0225, 0.,
      0., 0.0225;

  // measurement matrix
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0., 0., 0., 1.;

  // set the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // set the process covariance matrix Q
  Q_ = Eigen::MatrixXd(4, 4);
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

/**
 * Predicts the state estimate for the next time step using the Kalman Filter's prediction step.
 *
 * This function predicts the state estimate for the next time step based on the current state estimate 'x_' and state covariance 'P_', using the Kalman Filter's prediction step.
 *
 * @remark This function assumes that the Kalman Filter has already been initialized with the state transition matrix 'F_' and has a valid state estimate 'x_' and state covariance 'P_'.
 */
void KalmanFilter::predict()
{
  // Implement Kalman Filter Predict

  // Predict the state estimate 'x_'
  x_ = F_ * x_;

  // Predict the state covariance 'P_'
  P_ = F_ * P_ * F_.transpose();
}

/**
 * Updates the state estimate using the Kalman Filter.
 *
 * This function updates the state estimate based on the observed measurement vector 'z' using the Kalman Filter.
 *
 * @param z The observed measurement vector at the current time step.
 *
 * @remark This function assumes that the Kalman Filter has already been initialized with an initial state estimate 'x_', an initial state covariance 'P_', a measurement matrix 'H_', and a measurement noise covariance 'R_'.
 */
void KalmanFilter::update(const Eigen::VectorXd &z)
{
  // Implement Kalman Filter Update

  // Calculate the innovation 'y' (measurement residual)
  Eigen::VectorXd y = Eigen::VectorXd(z - (H_ * x_));

  // Calculate the innovation covariance 'S'
  Eigen::MatrixXd S = Eigen::MatrixXd(H_ * P_ * H_.transpose() + R_);

  // Calculate the Kalman Gain 'K'
  Eigen::MatrixXd K = Eigen::MatrixXd(P_ * H_.transpose() * S.inverse());

  // Update the state estimate 'x_'
  x_ = x_ + (K * y);

  // Identity matrix for size adjustment
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());

  // Update the state covariance 'P_'
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}