#include "fodom/synthetic/synthetic.h"

#include <cmath>
#include <fstream>
#include <memory>

namespace fodom {
namespace synthetic {

const Scalar a = 100.0;  // amplitude parameter
void SE3EightTrajectory::sampleTrajectory(Scalar t, Vector3_<Scalar>& pos,
                                          Vector3_<Scalar>& vel,
                                          Vector3_<Scalar>& acc) const {
  // lemniscate cartesian equations
  Scalar scaling = (2 * M_PI / T());
  Scalar theta = t * scaling;

  Scalar ctheta = std::cos(theta);
  Scalar stheta = std::sin(theta);
  Scalar stheta2 = stheta * stheta;
  Scalar ctheta2 = ctheta * ctheta;

  pos << a * Vector3_<Scalar>(ctheta, ctheta * stheta, 0.0) / (stheta2 + (1));
  vel << scaling * a *
             Vector3_<Scalar>((stheta * (stheta2 - 3)), -(3 * stheta2 - 1), 0) /
             ((stheta2 + 1) * (stheta2 + 1));
  acc << scaling * scaling * a *
             Vector3_<Scalar>(
                 (ctheta * (10 * ctheta2 + ctheta2 * ctheta2 - 8)) /
                     ((ctheta2 - 2) * (ctheta2 - 2) * (ctheta2 - 2)),
                 -2 * ctheta * stheta * (3 * ctheta2 + 2) /
                     ((stheta2 + 1) * (stheta2 + 1) * (stheta2 + 1)),
                 0);
}

const Scalar r = 30.0;
void SE3CircleTrajectory::sampleTrajectory(Scalar t, Vector3_<Scalar>& pos,
                                           Vector3_<Scalar>& vel,
                                           Vector3_<Scalar>& acc) const {
  Scalar scaling = (2 * M_PI / T());
  Scalar theta = t * scaling;

  Scalar ctheta = std::cos(theta);
  Scalar stheta = std::sin(theta);

  pos << r * Vector3_<Scalar>(ctheta, stheta, 0.0);
  vel << scaling * r * Vector3_<Scalar>(-stheta, ctheta, 0.0);
  acc << scaling * scaling * r * Vector3_<Scalar>(-ctheta, -stheta, 0.0);
}

const Scalar A = 3;
void SE3SineTrajectory::sampleTrajectory(Scalar t, Vector3_<Scalar>& pos,
                                         Vector3_<Scalar>& vel,
                                         Vector3_<Scalar>& acc) const {
  Scalar scaling = (2 * M_PI / T());
  Scalar theta = t * scaling;

  Scalar stheta = std::sin(theta);
  Scalar ctheta = std::cos(theta);

  pos << Vector3_<Scalar>(t, A * stheta, 0.0);
  vel << Vector3_<Scalar>(1, scaling * A * ctheta, 0.0);
  acc << Vector3_<Scalar>(0.0, -scaling * scaling * A * stheta, 0.0);
}

const Scalar l = 10;
const Scalar theta = 0;
const Scalar v0 = 0.1;
void SE3StraightTrajectory::sampleTrajectory(Scalar t, Vector3_<Scalar>& pos,
                                             Vector3_<Scalar>& vel,
                                             Vector3_<Scalar>& acc) const {
  Vector3_<Scalar> end(l * std::cos(theta), l * sin(theta), 0);
  Scalar scaling = 1 / T();
  Scalar s = t / T();
  Scalar s2 = s * s;
  Scalar s3 = s2 * s;
  Scalar s4 = s3 * s;
  Scalar s5 = s4 * s;

  pos << (6 * (1 - v0) * s5 - 15 * (1 - v0) * s4 + 10 * (1 - v0) * s3 +
          v0 * s) *
             (end);
  vel << (30 * (1 - v0) * s4 - 60 * (1 - v0) * s3 + 30 * (1 - v0) * s2 + v0) *
             (end)*scaling;
  acc << (120 * (1 - v0) * s3 - 180 * (1 - v0) * s2 + 60 * (1 - v0) * s) *
             (end)*scaling * scaling;

  // DEBUG version: imu measurements all zero
  // pos << s * (end);
  // vel << scaling * (end);
  // acc << scaling * scaling * 0 * (end);
}

void SE3PlanarTrajectory::getPoseMeasurement(
    Scalar t, Isometry3_<Scalar>& pose, Vector3_<Scalar>& vel,
    IMUMeasurement<Scalar>& measurement) const {
  pose.setIdentity();

  Vector3_<Scalar> pos, acc;

  sampleTrajectory(t, pos, vel, acc);

  // Pose

  pose.translation() << pos;
  // compute the tangent vector to the curve
  Scalar v_norm = vel.norm();

  if ((std::fabs(v_norm) < 1e-8))
    throw std::runtime_error("velocity appears to be zero ");
  Vector3_<Scalar> tangent = vel / v_norm;

  Vector3_<Scalar> z_vers(0, 0, 1);
  z_vers.normalize();

  Vector3_<Scalar> y_vers = z_vers.cross(tangent);
  y_vers.normalize();

  Matrix3_<Scalar> rotation;
  rotation.col(0) << tangent;
  rotation.col(1) << y_vers;
  rotation.col(2) << z_vers;

  pose.linear() = rotation;

  // linear acceleration in moving frame
  const Matrix3_<Scalar>& R = pose.rotation();
  measurement.lin_acc = R.transpose() * acc;

  // expecting only wz != 0
  measurement.ang_vel << 0, 0,
      (vel(0) * acc(1) - vel(1) * acc(0)) / (v_norm * v_norm);
  measurement.stamp = t;
}
void SE3PlanarTrajectory::generateData(
    Scalar freq,
    std::vector<std::tuple<IMUMeasurement<Scalar>, Vector3_<Scalar>,
                           Isometry3_<Scalar>>>& data) const {
  int num = T_ * freq;
  Scalar dt = 1 / freq;

  data.resize(num);

  Scalar t = 0;
  for (int i = 0; i < num; ++i, t += dt) {
    Isometry3_<Scalar>& pose = std::get<2>(data.at(i));
    IMUMeasurement<Scalar>& meas = std::get<0>(data.at(i));
    Vector3_<Scalar>& vel = std::get<1>(data.at(i));
    getPoseMeasurement(t, pose, vel, meas);
  }
}
void FakeImu::generateData(
    std::vector<std::tuple<IMUMeasurement<Scalar>, Vector3_<Scalar>,
                           Isometry3_<Scalar>>>& data,
    bool noise) {
  trajectory().generateData(freq_, data);

  // initialize biases
  Vector3_<Scalar> ba_ = 1e-8 * Vector3_<Scalar>::Ones();
  Vector3_<Scalar> bg_ = 1e-8 * Vector3_<Scalar>::Ones();

  std::normal_distribution<double> std_dist(0.0, 1.0);

  Scalar t = 0;
  for (int i = 0; i < data.size(); ++i) {
    Isometry3_<Scalar>& pose = std::get<2>(data.at(i));
    IMUMeasurement<Scalar>& meas = std::get<0>(data.at(i));
    Vector3_<Scalar>& vel = std::get<1>(data.at(i));
    // noise
    // maybe bugged
    if (noise) {
      // ba_ += std_bias_acc_ * Vector3_<Scalar>(std_dist(rnd_gen_),
      // std_dist(rnd_gen_), std_dist(rnd_gen_)); bg_ += std_bias_gyro_ *
      // Vector3_<Scalar>(std_dist(rnd_gen_), std_dist(rnd_gen_),
      // std_dist(rnd_gen_));
      meas.lin_acc += ba_ + std_acc_ * Vector3_<Scalar>(std_dist(rnd_gen_),
                                                        std_dist(rnd_gen_),
                                                        std_dist(rnd_gen_));
      meas.ang_vel += bg_ + std_gyro_ * Vector3_<Scalar>(std_dist(rnd_gen_),
                                                         std_dist(rnd_gen_),
                                                         std_dist(rnd_gen_));
    }
  }
}

void MeasurementTrajectory::generateData(
    Scalar freq,
    std::vector<std::tuple<IMUMeasurement<Scalar>, Vector3_<Scalar>,
                           Isometry3_<Scalar>>>& data) const {
  int k = 10;

  Scalar freq_int =
      k * freq;  // the frequency at which the measurements is integrated

  int num = T_ * freq_int;
  Scalar dt = 1 / freq_int;

  data.resize(T_ * freq);
  std::get<0>(data.at(0));
  std::get<1>(data.at(0)).setZero();
  std::get<2>(data.at(0)).setIdentity();

  Isometry3_<Scalar> curr_pose;
  Vector3_<Scalar> curr_vel;

  curr_pose.setIdentity();
  curr_vel.setZero();

  Scalar t = 0;
  int j = 0;
  for (int i = 0; i < num; ++i, t += dt) {
    Vector3_<Scalar> acceleration, angular_vel;
    sampleMeasurement(t, acceleration, angular_vel);

    if (i % k == 0) {
      IMUMeasurement<Scalar>& meas = std::get<0>(data.at(j));
      meas.lin_acc = acceleration;
      meas.ang_vel = angular_vel;
      meas.stamp = t;

      std::get<1>(data.at(j)) = curr_vel;
      std::get<2>(data.at(j)) = curr_pose;

      ++j;
    }
    // numerically integrate the pose given the measurements
    curr_pose.translation() += curr_vel * dt + 0.5 * acceleration * dt * dt;
    curr_vel += dt * acceleration;

    Matrix3_<Scalar> R_curr = curr_pose.linear();
    R_curr *= expMapSO3((angular_vel * dt).eval());

    fixRotation(R_curr);
    curr_pose.linear() = R_curr;
  }
}

void MeasurementTrajectory::sampleMeasurement(Scalar t, Vector3_<Scalar>& acc,
                                              Vector3_<Scalar>& ang_vel) const {
  if (t < T_ / 3) {
    acc << 1, 0, 0;
    ang_vel << 0, 0, 0;
  } else if (t < 2 * T_ / 3) {
    acc << 0, 0, 0;
    ang_vel << 0, 0, 0;
  } else {
    acc << 0, 0, 0;
    ang_vel << 0, 0, 0;
  }
}

}  // namespace synthetic
}  // namespace fodom