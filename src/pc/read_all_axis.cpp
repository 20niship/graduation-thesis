/**
 * @file read_all_axis.cpp
 * @brief libmodbusを用いてMaestroと接続し、すべてのモータとエンコーダの情報を読み取る
 * @date 2023/7/6 作成
 * tomlファイルでモータ等のデータの指定を行う
 */

#include "hr4c/common.hpp"

int main() {
  while(1) {
    if(kbhit()) break;
  }
  return 0;
}
