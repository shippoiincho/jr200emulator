# Matsushita JR-200 Emulator for Raspberry Pi Pico

# WORK IN PROGRESS

---
# 概要

松下の JR-200 のエミュレータです。
以下の機能を実装しています。

- メイン RAM (48KB)
- BEEP
- テープ
- Joypad



---
# 配線

Pico と VGA コネクタやブザー/スピーカーなどを以下のように配線します。

- GPIO0 VGA:H-SYNC
- GPIO1 VGA:V-SYNC
- GPIO2 VGA:Blue0 (330 Ohm)
- GPIO3 VGA:Blue1 (680 Ohm)
- GPIO4 VGA:Red0 (330 Ohm)
- GPIO5 VGA:Red1 (680 Ohm)
- GPIO6 VGA:Red2 (1.2K Ohm)
- GPIO7 VGA:Green0 (330 Ohm)
- GPIO8 VGA:Green1 (680 Ohm)
- GPIO9 VGA:Green2 (1.2K Ohm)
- GPIO10 Audio

このほかに VGA、Audio の　GND に Pico の　GND を接続してください。

---
# サウンド


---
# 使い方

`prebuild` 以下にある uf2 ファイルを Pico に書き込みます。

- msxemulator.uf2           PWM 出力(PSGのみ) 


初めて使う場合には、システム ROM などの書き込みが必要です。

---
# ROM など

著作権の関係で ROM は含まれていません。

用意したファイルを [picotool](https://github.com/raspberrypi/pico-sdk-tools/releases)などで書き込みます。

```
BASIC ROM
$ picotool load -v -x basic.rom -t bin -o 0x10030000

MONITOR ROM
$ picotool load -v -x monitor.rom -t bin -o 0x10038000

FONT 
$ picotool load -v -x font.rom -t bin -o 0x1003C000
```



---
# キーボード

Pico の USB 端子に、OTG ケーブルなどを介して USB キーボードを接続します。
USB キーボードに存在しないキーは以下のように割り当てています。

- STOP   → Pause/Break
- SELECT → ScrollLock
- カナ　　→ カタカナ・ひらがな
- GRAPH　→ ALT

また F12 でメニュー画面に移ります。
ROM ファイルや テープイメージの操作ができます。

---
# Joystick

DirectInput 対応のゲームパッド(1台)に対応しています。
ボタンの割り当ては、`joystick.c` で変更できます。

---
# Tape

CAS 形式ファイルの入出力に対応しています。
LittleFS 上においてください。

---
# VGA

メモリ節約のため、今回はフレームバッファ方式ではなく、1 ラインつづ CPU を使ってリアルタイムに描画しています。
なので、フラッシュへの書き込み操作時に画面更新が止まります。

---
# ライセンスなど

このエミュレータは以下のライブラリを使用しています。

- MAME の MC6800 エミュレータ
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)
- [HID Parser(おそらくLUFAの改変)](https://gist.github.com/SelvinPL/99fd9af4566e759b6553e912b6a163f9)

---
# 制限事項


- Gamepad を複数接続するとうまく操作できないことがあります

---
# Gallary


