
# 全体構成（モジュール）

```mermaid
graph TD;

subgraph Robot
r_motor[[モータ]]
r_sensor[[センサ]]
end

subgraph Simulator
s_motor[[モータ]]
s_sensor[[センサ]]
end

modbus[(modbus サーバー)]
r_motor --> modbus
r_sensor --> modbus
s_motor --> modbus
s_sensor --> modbus

subgraph Controller
教師データ生成>教師データ記録]
動かす>動かす]
学習モジュール
database[(教師データ)]
end

学習モジュール --学習モデル-->database 
教師データ生成 --> modbus
教師データ生成 --> database
動かす --> modbus


```

# タスク

- [ ] シミュレーション
  - [ ] ROSにモデル読み込み
  - [ ] 
- [ ] ハード制作
- [ ] コントローラ制作
  - [ ] 回路設計
  - [ ] modbusで通信する 
- [ ] 学習手法の確率


# スケジュール

| 月 | 内容 | 進捗 | 補足 |
| :---: | :--- | :---: | :--- |
| 5/19 | 本格スタート |  |  |
|5 |  |  |  |


