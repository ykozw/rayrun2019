# 日本レイトレーシング協会公式認定 全日本レイトレ選手権 交差判定部門

## ルール
- 高速なpreprocess()とintersect()を実装してください。
- ビルド済みのdllを提出してください。
- 実行は[c5.2xlarge](https://aws.amazon.com/jp/ec2/instance-types/c5/)のWindowsインスタンス上で行われます。

## 実装に関して
- preprocess()はシーンの構築を行います。
- preprocess()は複数回呼び出され、前回の構築データは破棄してください。
- intersect()はシーンに対する交差判定を行います。
- intersect()はpreprocess()の後に呼び出されます。
- intersect()は複数のスレッドから呼び出されます。

## 禁止事項
- GPUは使用しないでください。
- Embreeなどサードパーティの交差判定ライブラリは使用しないでください。
- 交差判定以外であればサードパーティライブラリは使用しても構いません。

## 対決方法
- 1on1のトーナメント方式です。
- レンダリングが先に終了したほうが勝ち抜きです。
- レンダリングモデル、シチュエーションを対決毎にじゃんけんで勝った方が選びます。

## 参考実装のビルド方法

```
mkdir rayrun
cd rayrun
git clone https://github.com/qatnonoil/rayrun2019.git
git submodule update --init
build.bat
```

## その他
- テストデータは[McGuire Computer Graphics Archive](https://casual-effects.com/data/)よりダウンロードしてください。