/*
 * communication_const.hpp
 * @brief modbusでPCとMaestro通信時のフォーマット定数を定義する
 * @date 2023/7/7
 * @auther 20niship
 */

#pragma once

namespace hr4c {

// MODBUS CONFIG
#define MODBUS_WRITE_IN_INDEX 0    // write in is regArr[MODBUS_WRITE_IN_INDEX]
#define MODBUS_WRITE_IN_CNT 60     // to regArr[MODBUS_WRITE_IN_INDEX + MODBUS_WRITE_IN_CNT -1]
#define MODBUS_READ_OUTPUT_INDEX 0 // read_out is regArr[MODBUS_READ_OUTPUT_INDEX]
#define MODBUS_READ_CNT 60         // to regArr[MODBUS_READ_OUTPUT_INDEX + MODBUS_READ_CNT -1]

// MODBUS ARRAY ID
// #define MODBUS_TIME_START_INDEX 0 // 0 <= id <8 is h, m, s, sec

enum mbusStartIdlist : int {
  eTimeStartId = 0, // regArr[eTimeStart] ~ regArr[eTimeStart+8] is Time

  eAx1 = 4,  // regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,
  eAx2 = 18, // regArr[eAx1] ~ regArr[eAx1+6] is pos1, vel1, tor1,

  eActualPos = 0,
  eActualVel = 2,
  eActualTor = 4,
  eActualCur = 6,
  eKpPos     = 8,
  eKpVel     = 10,
  eKiVel     = 12,

  eCommand1 = 0,
  eCommand2 = 2,
  eCommand3 = 4,
};

} // namespace hr4c
