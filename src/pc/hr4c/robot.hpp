#pragma once
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <hr4c/AxisInterface.hpp>
#include <hr4c/ModbusClient.hpp>
#include <hr4c/h4rc.hpp>

namespace hr4c {

struct Link {
  double mass;                  // リンクの質量 [kg]
  Eigen::Vector3d centerOfMass; // リンクの重心位置 [m]
  Eigen::Matrix3d inertia;      // リンクの慣性モーメント [kg・m^2]
};

struct Joint {
  std::string name;
  double position; // ジョイントの位置 [rad]
  double velocity; // ジョイントの速度 [rad/s]
  double torque;   // ジョイントにかかるトルク [Nm]

  AxisInterface axis; // ジョイントを制御するためのインターフェース
};

class Hr4cRobotBase {
private:
  std::vector<AxisInterface> axis;

public:
  Hr4cRobotBase();
  ~Hr4cRobotBase();
  // no copyable
  Hr4cRobotBase(Hr4cRobotBase& other)            = delete;
  Hr4cRobotBase(Hr4cRobotBase&& other)           = delete;
  Hr4cRobotBase& operator=(const Hr4cRobotBase&) = delete;
  Hr4cRobotBase& operator=(Hr4cRobotBase&&)      = delete;

  //! tomlファイルを読み込んでロボットデータを初期化
  bool load(const std::string& filepath);

  bool start();
  bool connect();
  bool read(uint16_t* tab_rp_registers, int nb);
  bool write(uint16_t* tab_rp_registers, int nb);
  bool disconnect();
};

} // namespace hr4c
