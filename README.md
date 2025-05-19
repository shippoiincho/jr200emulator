# Matsushita JR-200 Emulator for Raspberry Pi Pico

---
# 概要

松下の JR-200 のエミュレータです。
以下の機能を実装しています。

- メイン RAM (48KB)
- BEEP・サウンド
- テープ
- Joypad

---
# 配線

Pico と VGA コネクタやブザー/スピーカーなどを以下のように配線します。

- GPIO0 VGA:H-SYNC
- GPIO1 VGA:V-SYNC
- GPIO2 VGA:Blue  (330 Ohm)
- GPIO3 VGA:Red   (330 Ohm)
- GPIO4 VGA:Green (330 Ohm)

- GPIO6 Audio

このほかに VGA、Audio の　GND に Pico の　GND を接続してください。

---
# サウンド

内蔵タイマーによるサウンド出力(PLAY/SOUND 文)に対応しています。
GPIO6 に PWM で出力されます。

---
# 使い方

`prebuild` 以下にある uf2 ファイルを Pico に書き込みます。

- jr200emulator.uf2


初めて使う場合には、システム ROM などの書き込みが必要です。

---
# ROM など

著作権の関係で ROM は含まれていません。

用意したファイルを [picotool](https://github.com/raspberrypi/pico-sdk-tools/releases)などで書き込みます。

```
BASIC ROM
$ picotool load -v -x basic.rom -t bin -o 0x10020000

MONITOR ROM
$ picotool load -v -x monitor.rom -t bin -o 0x10028000

FONT 
$ picotool load -v -x font.rom -t bin -o 0x1002C000
```



---
# キーボード

Pico の USB 端子に、OTG ケーブルなどを介して USB キーボードを接続します。
USB キーボードに存在しないキーは以下のように割り当てています。

- RubOut → Backspace
- 英数    → CapsLock
- カナ    → カタカナ・ひらがな
- GRAPH  → 変換・前候補

また F12 でメニュー画面に移ります。
テープイメージの操作ができます。

---
# Joystick

DirectInput 対応のゲームパッド(1台)に対応しています。
ボタンの割り当ては、`joystick.c` で変更できます。

---
# Tape

Pico の LittleFS 上におかれます。
ファイル形式は独自フォーマットになっています。

ボーレートの設定は 2400 Baud になっています。
600 Baud の動作はサポートされていません。

---
# VGA

メモリ節約のため、フレームバッファ方式ではなく、1 ラインつづ CPU を使ってリアルタイムに描画しています。
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

- 起動直後の最初のキーが入力できないことがあります
- Gamepad を複数接続するとうまく操作できないことがあります
- CPU の未定義命令を使っているソフトは動作しません

---
# Gallary


