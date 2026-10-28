# 安定版のEmscripten SDKをベースにする
FROM emscripten/emsdk:3.1.64

# 必要なパッケージの追加（基本ツールはすでに入っていますが、明示的に確保）
RUN apt-get update && apt-get install -y \
    cmake \
    git \
    python3 \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Linux ネイティブ版のビルドとデバッグに使うパッケージ
#   libsdl2-*-dev  : ネイティブ版がリンクする SDL2 本体・画像・音声ライブラリ
#   gdb            : VS Code（cppdbg）がネイティブ版のデバッグに使うデバッガ
#   libgl1-mesa-dri: ソフトウェア OpenGL。これが無いと SDL_CreateRenderer が
#                    「Couldn't find matching render driver」で失敗する
RUN apt-get update && apt-get install -y \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libgl1-mesa-dev \
    gdb \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# 日本語のファイル名を扱えるようにする
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# コンテナ内の作業ディレクトリを設定
WORKDIR /workspace
