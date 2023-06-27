# Maestro package

## Ubuntuでビルドできるようにする
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


## 出力ファイル形式
WindowsのElmo Developper Studioで実行可能なバイナリをさくせいして、そのアーキテクチャを確認する：
```sh
$file test_project.pexe

test_project.pexe: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), dynamically linked, interpreter /lib/ld-linux-armhf.so.3, for GNU/Linux 2.6.31, BuildID[sha1]=470373317c5d8d41101320e18c8dc91f72dab173, with debug_info, not stripped
```



## connect
`ssh user@192.168.2.110` パスワードは`user`


