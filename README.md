# DRIFT

[ブラウザで遊ぶ](https://akitakann.github.io/drift-shift/) · [ソースコード](https://github.com/akitakann/drift-shift)

公開画面はゲームのみです。画面をクリックして開始し、P / Escでメニューを開けます。クレジットは開始・一時停止メニューから確認できます。

## 遊び方

鈴鹿サーキット1周のタイムアタックです。3・2・1のカウントダウンのあと、GOで計測が始まります。S字・デグナー・ヘアピン・スプーン・130R・シケインを通るたびに区間タイムが出て、自分のベストより速ければ画面の縁が緑に光ります。1周を走り切るとタイムが出て、そのまま次の周に続きます。ベストタイムはブラウザに保存されます。

Wで加速し、ハンドルを切りながらSpaceを押し続けるとドリフトします。タイヤの光が3段階で変わり、Spaceを離すとブーストします。ドリフト中も左右のハンドルで進路を調整できます。黄色い箱は、コーナーではドリフトのライン（外から入り、頂点でインに寄り、外へ抜ける）の上に、長い直線では3個ずつ左右交互に並んでいます。通ると短く加速し、ドリフト中に取るとチャージも進みます。箱は次の周で元に戻ります。

通常の最高速は200km/hです（画面の表示。実際の移動は表示の1.45倍の速さで、`moveSpeedScale` で変えられます。速さに合わせて、ハンドルの曲がりも `turnRateScale` で1.15倍にしています）。芝生では減速し、ドリフトの充電・ブーストを取り消します。小さな段差は吸収し、大きな起伏では勢いを残してジャンプします。

| 操作 | キー | コントローラー |
|---|---|---|
| アクセル | W / ↑ | RT |
| ブレーキ・後退 | S / ↓ | LT |
| ハンドル | A D / ← → | 左スティック・十字キー |
| ドリフトをためる・離してブースト | Space | A |
| 一時停止・再開 | P / Esc | Start |
| 最初から走る | R | Back |
| 音の切り替え | M | |

GO、黄色い箱、ブースト、区間ベスト、周回、新記録の瞬間には、画面の光・色ずれ・フラッシュ・スローの演出が入り、コントローラーが振動します（振動に対応したブラウザのみ）。

別の画面へ移ると一時停止します。戻って「再開」を押してください。全画面表示と音の切り替えもメニューで操作できます。

BGMは「Slam Van」と「Time Bomb（echo_orbitリミックス版）」を交互にフル再生し、曲間を2.5秒でクロスフェードします。Rキーやメニューから走り直しても曲の再生位置は維持します。一時停止中は曲もその位置で止まり、Mキーの音の切り替えはBGMにも適用されます。

## 公開版の設定

車・カメラ・音・演出の調整値は [web/release-preset.json](web/release-preset.json) に固定しています。公開画面に調整パネル・比較・評価・記録のUIはなく、以前の試作ページがブラウザに保存した調整値も読み込みません。

CMakeが設定をページへ組み込みます。設定を変更する場合はJSONを編集して再ビルドしてください。画面は黒い余白の中で16:9を保ち、車とカメラの構図を維持します。

## ビルド

C++ / SDL2 / OpenGLを使用しています。ブラウザ版はEmscripten 3.1.64でビルドします。

~~~sh
emcmake cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
python3 -m http.server 8080 --directory build
~~~

http://localhost:8080/ を開きます。GitHub Pagesへの公開はGitHub Actionsでブラウザ版をビルドして行います。

アイテムの取得音・風の音・アフターファイア・カウントダウン・区間・周回・新記録の効果音は `assets/audio/sfx/` にあり、`tools/sfx/build.sh` で作り直せます（Python 3・numpy・scipy・ffmpegが必要）。どれか読めないファイルがあれば、その音だけ以前の合成音で鳴ります。チャージ・ブースト・アイテムの加速はファイルではなく、エンジン音そのものの回転と大きさを上げて表しています（`src/audio.cpp`）。調整値は `web/release-preset.json` の `charge…`・`boost…`・`item…` です。

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
- BGM: [Slam Van — The Spin Wires](https://freemusicarchive.org/music/The_Spin_Wires/single/slam-van/)（[CC BY-NC 4.0・非商用限定](https://creativecommons.org/licenses/by-nc/4.0/)）。
- BGM: [Time Bomb — echo_orbit](https://ccmixter.org/files/echo_orbit/67147)、ボーカル [Thespinwires](https://ccmixter.org/files/Thespinwires/64088)（ともに[CC BY 3.0](https://creativecommons.org/licenses/by/3.0/)）。バンドの原曲録音ではなくecho_orbitのリミックス版を使用しています。
- BGMの取得元・加工・ライセンスは [web/music/CREDITS.md](web/music/CREDITS.md) に記載しています。音源それぞれの利用条件を保持してください。
- 音源は公開HQプレビューから取得し、WAV化・音量調整・ループ境界加工を行っています。タイヤ音にはハイパス処理も加えています。区間・加工・ライセンスの詳細は [assets/audio/CREDITS.md](assets/audio/CREDITS.md) を参照してください。エンジン録音は車の動きに応じて再生速度を変えています。
- コース中心線: © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright)、[鈴鹿サーキット relation 284570](https://www.openstreetmap.org/relation/284570)（[Open Database License](https://opendatacommons.org/licenses/odbl/1-0/)）。
- コース標高: 出典 [国土地理院](https://www.gsi.go.jp/)の標高タイル。
- 車両・演出、衝突・風・着地音はプログラムで生成しています。アイテム・アフターファイア・カウントダウン・区間・周回・新記録の効果音も `tools/sfx/` のスクリプトで合成しています。チャージ中とブーストのエンジンの吹け上がりは、上記のエンジン音（qubodup、CC BY 4.0）をゲーム内で音程を変えて鳴らしています。

本作は課題用の非公式ゲームです。
