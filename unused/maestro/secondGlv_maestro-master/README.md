# remoteSecondGlove
## Abstaract
- 2号機動かす用のスクリプト．Modbus通信が記述してある，Advanced Elmo C++ ProjectというTemplateを改造
- 処理の順番は.h参照．
- mbusStartIdlist（.h内）で通信する配列番号を定義．
32bitを通信したいのに，modbusは上限16bitなので，2**16で割った商と余りを送信
なので，エンコード・デコード用の関数があることにも注意．
- Axisを増やすときは，  
-- mbusStartIdlistに配列番号を追加  
-- CMMCSingleAxisに新しいaxisを定義（使わないときは余剰数合ってもbuildは通る  
-- TorControlsクラスで，トルク制御を定義(EASIIが吐いた値を参考に決定．修論参照) で出来る  
-- main文内に処理を追加：他のaxisのマネをしてください  

## TorControls.cpp
- 速度をP制御，指令電流をPI制御．
- Elmo DriverのQuick-tuningの結果を参考にすぐにゲインが決められる．
- また，電流にはthresholdも設けている．過電流を防ぐため．

## Session start
ISIwiki参照

## Tips
### During Development
- modbusの仕様に注意．サーバーレジスタとローカルレジスタは独立です．
- したがって， 「サーバーレジスタのid12番目から6文字ローカルレジスタへコピー」
とかやっても，ローカルレジスタの0番目から6番目にそれらは格納されます．
### Others
- powerOnAxis()の中にaxs.SetOpMode()があつが，バグなのか言うことを聞かない．
