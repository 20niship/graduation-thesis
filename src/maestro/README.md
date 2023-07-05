# Maestro package

## 概要
// TODO: 書け


## Ubuntuでビルドできるようにする

### 1. クロスコンパイラのインストール
- amd64bit上で動く、ターゲットがARMの32bit LSB(リトルエンディアン) 用のコンパイラを用意する
- `sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf`で最新版を入れることができるが、これだとElmo上のGLIBCのバージョンと適合しないので、古いバージョンを使う必要がある
- よって、Linaroが配布しているクロスコンパイラをダウンロードして使用する
  - 動作するバージョンのコンパイラ：
  - https://releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/
  - ダウンロードしたら`/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin`の中にある、`〇〇-g++`を使用する
  - 詳しくは、`CMakeLists.txt`を参照

### 2. 動作

```sh
cd src/maestro
mkdir build && cd build
cmake ..
make

# make writeすると、コンパルしたデータをElmoにscpで送ってくれる
make write
```
で、
```sh
ssh user@192.168.2.52
./main.pexe ....
```

### 3. 作業ログとエラー対処

#### Elmo Developper studioに含まれるコンパイラのバージョンは以下の通り：
```sh
# Elmo公式のg++のバージョン
$  C:\cygwin\opt\crosstool\gcc-linaro-arm-linux-gnu\bin\arm-linux-gnueabihf-g++.exe --version              11:51:31
arm-linux-gnueabihf-g++.exe (crosstool-NG linaro-1.13.1-4.7-2013.03-20130313 - Linaro GCC 2013.03) 4.7.3 20130226 (prerelease)
Copyright (C) 2012 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

# Elmo公式のgccバージョン
$  C:\cygwin\opt\crosstool\gcc-linaro-arm-linux-gnu\bin\arm-linux-gnueabihf-gcc.exe 
arm-linux-gnueabihf-gcc.exe (crosstool-NG linaro-1.13.1-4.7-2013.03-20130313 - Linaro GCC 2013.03) 4.7.3 20130226 (prerelease)
Copyright (C) 2012 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

#### コンパイラが違う場合のエラー
- 普通にリンクしようとすると、`relocations in generic ELF (EM: 20)`って言われる
- これはaファイルのコンピュータArchitextureとホストPC（実行するPC、作成したいバイナリファイル）のアーキテクチャが一致していないため起こる問題らしい
- .aファイルを見てみたら、これがPowerPCのELFであることがわかった。
  -  https://stackoverflow.com/questions/3740379/how-can-i-get-the-architecture-of-a-a-file

```sh
$ readelf -h ../GMAS-elmo-lib/lib/libMMCPPLIB.a

ファイル: ../GMAS-elmo-lib/lib/libMMCPPLIB.a(stdafx.o)
ELF ヘッダ:
  マジック:   7f 45 4c 46 01 02 01 00 00 00 00 00 00 00 00 00
  クラス:                            ELF32
  データ:                            2 の補数、ビッグエンディアン
  Version:                           1 (current)
  OS/ABI:                            UNIX - System V
  ABI バージョン:                    0
  型:                                REL (再配置可能ファイル)
  マシン:                            PowerPC
  バージョン:                        0x1
  エントリポイントアドレス:               0x0
  プログラムヘッダ始点:          0 (バイト)
  セクションヘッダ始点:          140 (バイト)
  フラグ:                            0x0                                                                                                                                                      
  Size of this header:               52 (bytes)
  Size of program headers:           0 (bytes)
  Number of program headers:         0
  Size of section headers:           40 (bytes)
  Number of section headers:         9
  Section header string table index: 6

```

- ということで、PowerPC用のクロスコンパイラ（Toolchain）を入れる（CMakeLists参照）
- で完了

↓fileコマンドで実行ファイルの適合アーキテクチャを確認できて、この形式にすると動くにゃ
```sh
$file test_project.pexe

test_project.pexe: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-armhf.so.3, for GNU/Linux 2.6.31, BuildID[sha1]=470373317c5d8d41101320e18c8dc91f72dab173, with debug_info, not stripped
```
