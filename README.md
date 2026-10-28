# DRIFT

[ブラウザで遊ぶ](https://akitakann.github.io/drift-shift/) · [ソースコード](https://github.com/akitakann/drift-shift)

公開画面はゲームのみです。画面をクリックして開始し、P / Escでメニューを開けます。クレジットは開始・一時停止メニューから確認できます。

## 遊び方

Wで加速し、ハンドルを切りながらSpaceを押し続けるとドリフトします。タイヤの光が3段階で変わり、Spaceを離すとブーストします。ドリフト中も左右のハンドルで進路を調整できます。

通常の最高速は200km/hです。芝生では減速し、ドリフトの充電・ブーストを取り消します。小さな段差は吸収し、大きな起伏では勢いを残してジャンプします。時間制限や周回数による終了はありません。

| 操作 | キー |
|---|---|
| アクセル | W / ↑ |
| ブレーキ・後退 | S / ↓ |
| ハンドル | A D / ← → |
| ドリフトをためる・離してブースト | Space |
| 一時停止・再開 | P / Esc |
| 最初から走る | R |
| 音の切り替え | M |

別の画面へ移ると一時停止します。戻って「再開」を押してください。全画面表示と音の切り替えもメニューで操作できます。

## 公開版の設定

車・カメラ・音の調整値は [web/release-preset.json](web/release-preset.json) に固定しています。公開画面に調整パネル・比較・評価・記録のUIはなく、以前の試作ページがブラウザに保存した調整値も読み込みません。

CMakeが設定をページへ組み込みます。設定を変更する場合はJSONを編集して再ビルドしてください。画面は黒い余白の中で16:9を保ち、車とカメラの構図を維持します。

## ビルド

C++ / SDL2 / OpenGLを使用しています。ブラウザ版はEmscripten 3.1.64でビルドします。

~~~sh
emcmake cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
python3 -m http.server 8080 --directory build
~~~

http://localhost:8080/ を開きます。GitHub Pagesへの公開はGitHub Actionsでブラウザ版をビルドして行います。

Linuxでのビルドと検証:

~~~sh
cmake -S . -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j2
./build-native/racegame --self-test
ctest --test-dir build-native --output-on-failure
~~~

## 出典・権利表記

- エンジン音: [Car Engine Loop — qubodup](https://freesound.org/people/qubodup/sounds/147242/)（[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)）。
- タイヤ音: [Chrysler LHS tire squeal 04 — audible-edge](https://freesound.org/people/audible-edge/sounds/71739/)（[CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)）。
- 音源は公開HQプレビューから取得し、WAV化・音量調整・ループ境界加工を行っています。タイヤ音にはハイパス処理も加えています。区間・加工・ライセンスの詳細は [assets/audio/CREDITS.md](assets/audio/CREDITS.md) を参照してください。エンジン録音は車の動きに応じて再生速度を変えています。
- コース中心線: © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright)、[鈴鹿サーキット relation 284570](https://www.openstreetmap.org/relation/284570)（[Open Database License](https://opendatacommons.org/licenses/odbl/1-0/)）。
- コース標高: 出典 [国土地理院](https://www.gsi.go.jp/)の標高タイル。
- 車両・演出、チャージ・ブースト・衝突・風・着地音はプログラムで生成しています。

本作は課題用の非公式ゲームです。
