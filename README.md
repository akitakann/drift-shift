# racegame

鈴鹿サーキットを走る、3D レースゲームのひな形です。車は 1 台です。ドリフトは入っていません。

コースの線形は OpenStreetMap（relation 284570、Suzuka International Racing Course）、高さは国土地理院の標高タイルです。

- 地図データ: © OpenStreetMap contributors
- 標高: 出典 国土地理院

画像は `assets/graphics/`、音声は `assets/sound/` に置きます。コースデータは `assets/track/suzuka.txt` です。

## 開き方

Cursor または VS Code でこのフォルダを開き、**Reopen in Container** を選びます。初回はイメージのビルドに数分かかります。

## 操作

キーボードとゲームパッドは、一般的な 3D レースゲームと同じです。ハンドブレーキはありません。

| 操作 | キーボード | ゲームパッド |
|---|---|---|
| アクセル | W / ↑ | RT（R2）。十字キー上でも可 |
| ブレーキ / 後退 | S / ↓ | LT（L2）。十字キー下でも可 |
| ステアリング | A D / ← → | 左スティック、十字キー左右 |
| リスタート | R | Back（View / Share） |
| カメラ | C | Y（△） |
| ポーズ | P | Start（Menu / Options） |
| 操作表示 | H | |

画面は追従、近め、遠めの 3 つです。ギアはオートマです。

## ドリフトの課題

滑らない動きは `Car::stepGrip`（`src/car.cpp`）だけです。速度は車体の前を向き、横滑りはしません。ドリフトはこの関数を差し替えて作ります。アクセル、ブレーキ、壁、路面の高さは `Car::update` 側です。

## ブラウザ版

```bash
emcmake cmake -B build
cmake --build build
python3 -m http.server 8080
```

`http://localhost:8080/build/racegame.html`

## Linux 版

```bash
cmake -B build-native
cmake --build build-native
./build-native/racegame
```

画面は `http://localhost:6080`（パスワード `vscode`）です。コンテナ内では音は鳴りません。

ロジックだけの確認:

```bash
./build-native/racegame --self-test
```
