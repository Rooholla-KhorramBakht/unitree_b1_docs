#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

class imuModel
{
public:
  // imuModel(const std::string& gyro_calib_file, const std::string& accel_calib_file)
  imuModel()
  {
    // std::tie(Ta, Ka, ba) = extractCalibParams(accel_calib_file);
    // std::tie(Tg, Kg, bg) = extractCalibParams(gyro_calib_file);
    Ta << 1, -0.00166593, 0.0345837, 0, 1, 0.00660962, -0, 0, 1;
    Ka << 0.00181168, 0, 0, 0, 0.00181232, 0, 0, 0, 0.00180771;
    ba << 32715.9, 32718.4, 32667.8;

    Tg << 1, 0.0432007, 0.00921694, 0.0127205, 1, 0.00360858, -0.0332311, 0.0383767, 1;
    Kg << 0.000531764, 0, 0, 0, 0.000524443, 0, 0, 0, 0.000529117;
    bg << 32770.3, 32762.8, 32769.6;
  }

  std::tuple<Vector3d, Vector3d> compute(const Vector3d& accel_raw, const Vector3d& gyro_raw)
  {
    Vector3d acc_calib = (Ta * Ka * (accel_raw + Vector3d(32768, 32768, 32768) - ba)).transpose();
    Vector3d gyro_calib = (Tg * Kg * (gyro_raw + Vector3d(32768, 32768, 32768) - bg)).transpose();
    return std::make_tuple(acc_calib, gyro_calib);
  }

private:
  // This function is to be fixed
  std::tuple<MatrixXd, MatrixXd, MatrixXd> extractCalibParams(const std::string& calibration_file)
  {
    std::ifstream file(calibration_file);
    std::vector<std::string> data;
    std::string line;
    while (std::getline(file, line))
    {
      if (!line.empty())
        data.push_back(line);
    }
    std::vector<MatrixXd> float_data;
    for (const std::string& data_line : data)
    {
      std::istringstream iss(data_line);
      std::vector<double> values;
      double value;
      while (iss >> value)
        values.push_back(value);
      float_data.emplace_back(Map<MatrixXd>(values.data(), values.size() / 3, 3));
    }
    MatrixXd T = float_data[0].rows() > 0 ? float_data[0] : MatrixXd::Zero(0, 0);
    MatrixXd K = float_data[1].rows() > 0 ? float_data[1] : MatrixXd::Zero(0, 0);
    MatrixXd b = float_data[2].rows() > 0 ? float_data[2] : MatrixXd::Zero(0, 0);
    return std::make_tuple(T, K, b);
  }

  Matrix3d Ta, Tg, Ka, Kg;
  Vector3d ba, bg;
};
