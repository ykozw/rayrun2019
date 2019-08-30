# 日本レイトレーシング協会公式認定 全日本レイトレ選手権 交差判定部門

## ルール
- 高速なpreprocess()とintersect()を実装してください。
- ビルド済みのdllを提出してください。
- 実行は[c5.2xlarge](https://aws.amazon.com/jp/ec2/instance-types/c5/)のWindowsインスタンス上で行われます。

## 実装に関して
- preprocess()はシーンの構築を行います。
- intersect()はシーンに対する交差判定を行います。
- intersect()はpreprocess()の後に呼び出されます。
- intersect()は複数のスレッドから呼び出されます。
- intersect()を複数スレッドから呼び出されたくない場合はneverUseOpenMP()を実装し、trueを返してください。

## 禁止事項
- GPUは使用しないでください。
- Embreeなどサードパーティの交差判定ライブラリは使用しないでください。
- 交差判定以外であればサードパーティライブラリは使用しても構いません。

## 対決方法
- 1on1のトーナメント方式です。
- レンダリングが先に終了したほうが勝ち抜きです。
- 対決前に早押しクイズを行います。勝利した方がレンダリングシチュエーションを選べます。
- レンダリングシチュエーションは当日までヒミツです。
- レンダリングモデルはMcGuireのページ以外からも用意されます。
- よくあるシチュエーションをはじめ、極端なシチュエーションも用意しています。
- レンダリングが60秒を超えた場合は自動的に不戦杯です。

## 参考実装のビルド方法

```
mkdir rayrun
cd rayrun
git clone https://github.com/qatnonoil/rayrun2019.git
git submodule update --init
build.bat
```

1. 出来たrayrun.slnを開きます。
2. rayrunプロジェクトの実行引数に"refimp "../asset/hairball.json""と入力します。
3. テスト実装(refimp.dll)が動きます。

このテスト実装の中身を参考に各自の実装を作ってください。

## その他
- テストデータは[McGuire Computer Graphics Archive](https://casual-effects.com/data/)よりダウンロードしてください。