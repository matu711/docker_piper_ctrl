# 双腕PiPER 教示データのLeRobot変換 README

本READMEでは、双腕PiPERのリーダーフォロワーで収集したMCAPデータを、収集用PC（`dnp218`）からチューニング用PC（`rlsv`）へ転送し、LeRobot形式に変換するまでの手順を説明する。

## 1. 収集用PCでのデータ収集・整理

### 1.1 リーダーフォロワーによるデータ収集

収集用PCで双腕PiPERのリーダーフォロワーを起動し、教示データを収集する。収集したMCAPファイルは、以下のディレクトリに保存される。

```text
/home/dnp218/docker_piper_env/piper_dataset
```

各エピソードは収集日時を含むファイル名で保存される。

**保存例**

```text
piper_dataset/
├── 20261015_153856.mcap
├── 20261015_154251.mcap
├── 20261015_155023.mcap
└── 20261015_160412.mcap
```

### 1.2 収集したエピソードを1つのフォルダにまとめる

**収集用PC（`dnp218`）** のターミナルで実行する。

```bash
cd /home/dnp218/docker_piper_env/piper_dataset
mkdir -p dnp1_20261015
mv 20261015_*.mcap dnp1_20261015/
```

`20261015` は実際の収集日に変更する。ここではデータセット名を `dnp1_20261015` としている。

### 1.3 整理結果の確認

```bash
ls -lh /home/dnp218/docker_piper_env/piper_dataset/dnp1_20261015/
```

**整理後の構成例**

```text
piper_dataset/
└── dnp1_20261015/
    ├── 20261015_153856.mcap
    ├── 20261015_154251.mcap
    ├── 20261015_155023.mcap
    └── 20261015_160412.mcap
```

## 2. チューニング用PCへのデータ転送

収集用PCで整理したMCAPデータを、`scp` コマンドでチューニング用PC（`rlsv`）へ転送する。

### 2.1 転送先ディレクトリ

```text
/home/rlsv/project/datasets/mcap
```

### 2.2 SCPによるデータ転送

**収集用PC（`dnp218`）のターミナル** で、以下のコマンドを実行する。

```bash
scp -r /home/dnp218/docker_piper_env/piper_dataset/dnp1_20261015 \
rlsv@163.221.133.176:/home/rlsv/project/datasets/mcap/
```

- `scp`：SSHを利用したファイル転送コマンド
- `-r`：フォルダ内のファイルをまとめて転送
- `dnp1_20261015`：転送するデータフォルダ
- `rlsv@163.221.133.176`：チューニング用PCの接続先

実行後、パスワードを要求された場合はチューニング用PCのユーザーパスワードを入力する。転送先の `mcap` ディレクトリが存在することを事前に確認する。

## 3. チューニング用PCへのSSH接続と転送データの確認

### 3.1 SSH接続

データ転送後、**収集用PC（`dnp218`）またはローカルPCのターミナル**で、チューニング用PC（`rlsv`）にSSH接続する。

```bash
ssh rlsv@163.221.133.176
```

初回接続時は接続先のホスト鍵を確認し、問題がなければ `yes` と入力する。パスワードが求められた場合は、`rlsv` のパスワードを入力する。

接続後、以下でログイン先を確認する。

```bash
whoami
hostname
```

`whoami` の出力が `rlsv` であることを確認する。

### 3.2 転送データの確認

SSH接続した **チューニング用PC（`rlsv`）** のターミナルで、以下のコマンドを実行する。

```bash
ls -lh /home/rlsv/project/datasets/mcap/dnp1_20261015/
```

**転送後の構成例**

```text
/home/rlsv/project/datasets/mcap/
└── dnp1_20261015/
    ├── 20261015_153856.mcap
    ├── 20261015_154251.mcap
    ├── 20261015_155023.mcap
    └── 20261015_160412.mcap
```

すべてのMCAPファイルが転送されていることを確認する。


## 4. Dockerコンテナへの接続

### 4.1 ターミナルの準備

SSH接続先のターミナルで操作する。必要に応じて、GUI環境では `Terminator` を起動してもよい。

### 4.2 DockerコンテナIDの確認

起動中のDockerコンテナを一覧表示する。

```bash
docker ps
```

**出力例**

```text
CONTAINER ID   IMAGE                STATUS        NAMES
2e6f346ff7ff   piper-pi0-sim:111    Up 5 weeks    piper_pi0-pi0-sim-run
```

学習・データ変換に使用するコンテナの **`CONTAINER ID`** を確認する。

### 4.3 Dockerコンテナに入る

```bash
docker exec -it <CONTAINER ID> bash
```

**実行例**

```bash
docker exec -it 2e6f346ff7ff bash
```

以降のLeRobot変換コマンドは、Dockerコンテナ内で実行する。

**注意：** `docker exec` に指定するのはDockerイメージIDではなく、`docker ps` で確認したコンテナIDである。

## 5. MCAPからLeRobot形式へのデータ変換

チューニング用PCへ転送したMCAPデータを、π0.5の学習で使用するLeRobot形式に変換する。

### 5.1 変換スクリプト

```text
/workspace/piper_pi0/real/collect/convert_bimanual_to_lerobot.py
```

### 5.2 変換コマンドの実行

**チューニング用PC（`rlsv`）のDockerコンテナ内** で、以下のコマンドを実行する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/real/collect/convert_bimanual_to_lerobot.py \
  --input /workspace/datasets/mcap/dnp1_20261015 \
  --output /workspace/datasets/lerobot/local/dnp1_20261015 \
  --repo-id local/dnp1_20261015 \
  --gap-policy recover \
  --image-policy flag \
  --image-tolerance-ms 100
```

`dnp1_20261015` は実際のデータセット名に変更する。

| オプション | 説明 |
| --- | --- |
| `--input` | 変換元MCAPフォルダ |
| `--output` | LeRobot形式の出力先 |
| `--repo-id` | データセット識別名 |
| `--gap-policy recover` | 欠損フレームの復元処理を行う |
| `--image-policy flag` | 画像の対応付けに問題があるフレームをフラグで記録する |
| `--image-tolerance-ms 100` | 画像と基準時刻の許容時間差を100 msに設定する |

**注意：** 上記の `/workspace/datasets` はDockerコンテナ内のパスである。ホストPCの `/home/rlsv/project/datasets` が `/workspace/datasets` にマウントされていることを前提とする。実行前にコンテナ内から入力データが見えることを確認する。

```bash
ls -lh /workspace/datasets/mcap/dnp1_20261015/
```

### 5.3 変換結果の確認

変換コマンドが**正常終了した後**、以下を実行する。

```bash
ls -lh /workspace/datasets/lerobot/local/dnp1_20261015/
```

LeRobot形式のデータファイルやメタデータが生成されていることを確認する。変換処理中は出力先がまだ作成されていない場合があるため、完了前に出力先が見つからなくても、それだけで失敗とは判断しない。

**ディレクトリ構成例（Dockerコンテナ内）**

```text
/workspace/datasets/
├── mcap/
│   └── dnp1_20261015/         # 変換元MCAPデータ
└── lerobot/
    └── local/
        └── dnp1_20261015/     # 変換後LeRobotデータ
```

以上で、双腕PiPERの収集データをLeRobot形式へ変換する手順は完了となる。
