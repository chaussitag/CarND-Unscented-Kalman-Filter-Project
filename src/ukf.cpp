#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

static const double EPS = 0.000001;
/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.55;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.35;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  time_us_ = 0LL;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  n_sigma_points_ = 2 * n_aug_ + 1;

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_points_);

  weights_ = VectorXd(n_sigma_points_);
  // setup the weights
  weights_.fill(1.0 / (2.0 * (lambda_ + n_aug_)));
  weights_(0) *= 2.0 * lambda_;

  // initialize dimentions of the intermediate variables
  x_aug_ = VectorXd(n_aug_);
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  Xsig_aug_ = MatrixXd(n_aug_, n_sigma_points_);

  n_radar_ = 3;
  radar_sig_ = MatrixXd(n_radar_, n_sigma_points_);
  z_radar_pred_ = VectorXd(n_radar_);
  S_radar_ = MatrixXd(n_radar_, n_radar_);
  T_radar_ = MatrixXd(n_x_, n_radar_);

  n_lidar_ = 2;
  lidar_sig_ = MatrixXd(n_lidar_, n_sigma_points_);
  z_lidar_pred_ = VectorXd(n_lidar_);
  S_lidar_ = MatrixXd(n_lidar_, n_lidar_);
  T_lidar_ = MatrixXd(n_x_, n_lidar_);

  // change 'enable_nis_calculation_' to false to disbale NIS caculation
  enable_nis_calculation_ = true;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // skip the measurement if it should be ignored
  auto sensor_type = meas_package.sensor_type_;
  if ((!use_radar_ && sensor_type == MeasurementPackage::RADAR) ||
      (!use_laser_ && sensor_type == MeasurementPackage::LASER)) {
    return;
  }

  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      auto ro = meas_package.raw_measurements_(0);
      auto phi = meas_package.raw_measurements_(1);
      x_(0) = ro * cos(phi);
      x_(1) = ro * sin(phi);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }
    x_(2) = 0.0;
    x_(3) = 0.0;
    x_(4) = 0.0;

    // initialize the state covariance matrix P_
    P_.setIdentity();

    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // The prediction step
  //compute the time elapsed between the current and previous measurements in seconds
  auto dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  Prediction(dt);

  // the update step
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }

  time_us_ = meas_package.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // generate augmented sigma-points
  GenerateAugmentedSigmaPoints();

  // predict the generated sigma-points
  PredictSigmaPoints(delta_t);

  //predict state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    VectorXd diff = VectorXd(n_x_);
    diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    diff(3) = NormalizeAngle(diff(3));

    P_ += weights_(i) * diff * diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // predict lidar measurements
  PredictLidarMeasurement();

  // callculate cross correlation matrix
  T_lidar_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    VectorXd z_diff = lidar_sig_.col(i) - z_lidar_pred_;

    T_lidar_ += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd S_lidar_inverse = S_lidar_.inverse();
  // calculate Kalman gain
  MatrixXd K_lidar = T_lidar_ * S_lidar_inverse;

  VectorXd z_lidar(n_lidar_);
  z_lidar << meas_package.raw_measurements_(0),
             meas_package.raw_measurements_(1);

  //update state mean and covariance matrix
  VectorXd z_diff = z_lidar - z_lidar_pred_;
  x_ += K_lidar * z_diff;
  P_ -= K_lidar * S_lidar_ * K_lidar.transpose();

  // calculate lidar NIS
  if (enable_nis_calculation_) {
    double lidar_nis = z_diff.transpose() * S_lidar_inverse * z_diff;
    printf("NIS: lidar %f\n", lidar_nis);
    fflush(stdout);
  }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // predict radar measurements
  PredictRadarMeasurement();

  //calculate cross correlation matrix
  T_radar_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    VectorXd z_diff = radar_sig_.col(i) - z_radar_pred_;
    z_diff(1) = NormalizeAngle(z_diff(1));

    T_radar_ += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd S_radar_inverse = S_radar_.inverse();
  //calculate Kalman gain
  MatrixXd K_radar = T_radar_ * S_radar_inverse;

  VectorXd z_radar(n_radar_);
  z_radar << meas_package.raw_measurements_(0),
             meas_package.raw_measurements_(1),
             meas_package.raw_measurements_(2);
  //update state mean and covariance matrix
  VectorXd z_diff = z_radar - z_radar_pred_;
  z_diff(1) = NormalizeAngle(z_diff(1));

  x_ += K_radar * z_diff;
  P_ -= K_radar * S_radar_ * K_radar.transpose();

  // calculate radar NIS
  if (enable_nis_calculation_) {
    double radar_nis = z_diff.transpose() * S_radar_inverse * z_diff;
    printf("NIS: radar %f\n", radar_nis);
    fflush(stdout);
  }
}

void UKF::GenerateAugmentedSigmaPoints() {
  // generate augmented sigma-points
  x_aug_.head(n_x_) = x_;
  x_aug_(n_x_) = x_aug_(n_x_ + 1) = 0.0;

  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_, n_x_) = std_a_ * std_a_;
  P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug_.llt().matrixL();

  Xsig_aug_.col(0) = x_aug_;
  double s = sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug_.col(i + 1) = x_aug_ + s * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - s * L.col(i);
  }
}

void UKF::PredictSigmaPoints(double delta_t) {
  double half_dt_square;
  double v_k;
  double sai_k;
  double sai_dot_k;
  double cos_sai_k;
  double sin_sai_k;
  double delta_sai;
  MatrixXd dx1(n_x_, 1);
  MatrixXd dx2(n_x_, 1);
  for (int i = 0; i < n_sigma_points_; ++i) {
    half_dt_square = 0.5 * delta_t * delta_t;
    v_k = Xsig_aug_(2, i);
    sai_k = Xsig_aug_(3, i);
    sai_dot_k = Xsig_aug_(4, i);
    cos_sai_k = cos(sai_k);
    sin_sai_k = sin(sai_k);
    delta_sai = sai_dot_k * delta_t;

    dx2 << half_dt_square * cos_sai_k * Xsig_aug_(5, i),
        half_dt_square * sin_sai_k * Xsig_aug_(5, i),
        delta_t * Xsig_aug_(5, i),
        half_dt_square * Xsig_aug_(6, i),
        delta_t * Xsig_aug_(6, i);
    if (sai_dot_k < EPS) {
      dx1 << v_k * cos_sai_k * delta_t,
          v_k * sin_sai_k * delta_t,
          0.0,
          sai_dot_k * delta_t,
          0.0;
    } else {
      dx1 << (v_k / sai_dot_k) * (sin(sai_k + delta_sai) - sin(sai_k)),
          (v_k / sai_dot_k) * (-cos(sai_k + delta_sai) + cos(sai_k)),
          0.0,
          sai_dot_k * delta_t,
          0.0;
    }
    Xsig_pred_.col(i) = Xsig_aug_.col(i).head(n_x_) + dx1 + dx2;
  }
}

void UKF::PredictRadarMeasurement() {
  // transform the predicted state sigma-points to radar measurements
  for (int i = 0; i < n_sigma_points_; ++i) {
    const double px = Xsig_pred_(0, i);
    const double py = Xsig_pred_(1, i);
    const double v = Xsig_pred_(2, i);
    const double sai = Xsig_pred_(3, i);
    const double ro = sqrt(px * px + py * py);
    radar_sig_(0, i) = ro;
    radar_sig_(1, i) = atan2(py, px);
    radar_sig_(2, i) = v * (px * cos(sai) + py * sin(sai)) / ro;
  }

  //calculate mean predicted radar measurement
  z_radar_pred_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    z_radar_pred_ += weights_(i) * radar_sig_.col(i);
  }

  //calculate innovation covariance matrix S
  S_radar_.fill(0.0);
  S_radar_(0, 0) = std_radr_ * std_radr_;
  S_radar_(1, 1) = std_radphi_ * std_radphi_;
  S_radar_(2, 2) = std_radrd_ * std_radrd_;
  for (int i = 0; i < n_sigma_points_; ++i) {
    VectorXd z_radar_diff = radar_sig_.col(i) - z_radar_pred_;
    // normalize the angle
    z_radar_diff(1) = NormalizeAngle(z_radar_diff(1));
    S_radar_ += weights_(i) * z_radar_diff * z_radar_diff.transpose();
  }
}

void UKF::PredictLidarMeasurement() {
  // transform the predicted state sigma-points to lidar measurements
  for (int i = 0; i < n_sigma_points_; ++i) {
    lidar_sig_(0, i) = Xsig_pred_(0, i);
    lidar_sig_(1, i) = Xsig_pred_(1, i);
  }

  // calculate mean predicted lidar measurement
  z_lidar_pred_.fill(0.0);
  for (int i = 0; i < n_sigma_points_; ++i) {
    z_lidar_pred_ += weights_(i) * lidar_sig_.col(i);
  }

  // caculate innovation covariance matrix S
  S_lidar_.fill(0.0);
  S_lidar_(0, 0) = std_laspx_ * std_laspx_;
  S_lidar_(1, 1) = std_laspy_ * std_laspy_;
  for (int i = 0; i < n_sigma_points_; ++i) {
    VectorXd z_lidar_diff = lidar_sig_.col(i) - z_lidar_pred_;
    S_lidar_ += weights_(i) * z_lidar_diff * z_lidar_diff.transpose();
  }
}

double UKF::NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}
