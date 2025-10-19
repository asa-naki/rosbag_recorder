# ROS2 Rosbag Recorder

ROS2 Humble/Jazzy対応のrosbagレコーダーパッケージです。指定したSSDへの記録、毎分のファイル分割、topic/serviceの記録、YAML設定、正規表現によるtopic選択をサポートします。

## ROS2ディストリビューション対応

- **ROS2 Humble**: Topic記録のみサポート（Service記録は未対応）
- **ROS2 Jazzy**: Topic + Service記録フルサポート

パッケージは自動的にROS_DISTROを検出し、適切な機能を有効化します。

## 機能

- **SSD指定記録**: 指定したディレクトリ（SSD）への記録
- **時間分割**: 指定した間隔（分単位）でバッグファイルを自動分割
- **ROS2 Jazzy対応**: TopicとServiceの両方を記録可能
- **サービス制御**: ROS2サービスによる記録開始/停止
- **YAML設定**: 記録対象の設定をYAMLファイルで管理
- **正規表現サポート**: Topic名の正規表現マッチング
- **柔軟な設定**: パラメータまたはYAMLファイルでの設定
- **エラーハンドリング**: SSD未接続等の記録エラーを自動検出
- **ステータス監視**: リアルタイムでの記録状態監視
- **Foxglove対応**: 可視化ダッシュボードでの状態確認

## インストール

```bash
cd ~/workspace/stride_ws
colcon build --packages-select rosbag_recorder
source install/setup.bash
```

## 使用方法

### 1. YAML設定ファイルを使用する場合

```bash
# デフォルト設定で起動
ros2 launch rosbag_recorder rosbag_recorder.launch.py

# カスタム設定で起動
ros2 launch rosbag_recorder rosbag_recorder.launch.py \
    config_file:=my_config.yaml \
    output_directory:=/media/my_ssd/recordings \
    split_duration_minutes:=5
```

### 2. パラメータのみで設定する場合

```bash
ros2 launch rosbag_recorder rosbag_recorder_params.launch.py \
    output_directory:=/media/ssd/rosbag_recordings \
    split_duration_minutes:=2 \
    topics:='["/camera/image_raw", "/lidar/points", "/tf"]' \
    topic_regex_patterns:='["/sensor_.*", "/robot/.*"]'
```

### サービス操作

#### 記録開始
```bash
ros2 service call /rosbag/start_recording std_srvs/srv/Trigger
```

#### 記録停止
```bash
ros2 service call /rosbag/stop_recording std_srvs/srv/Trigger
```

## Foxglove での監視

### ダッシュボードのインポート

1. Foxglove Studioを起動
2. `File` → `Import layout` を選択  
3. `foxglove/rosbag_recorder_dashboard.json` をインポート

### 監視可能な項目

- **記録ステータス**: 現在記録中かどうかをリアルタイム表示
- **ストレージ健康状態**: SSDの接続状態とエラー検出
- **ディスク使用量**: 記録先ディスクの使用率をゲージ表示
- **記録統計**: ファイル数、バイト数、継続時間のグラフ
- **詳細情報**: 現在のファイルパス、エラーメッセージ等

詳細は `foxglove/README.md` を参照してください。