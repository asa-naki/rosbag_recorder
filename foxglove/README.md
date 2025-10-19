# Foxglove での ROS2 Rosbag Recorder 監視

このディレクトリには、Foxglove Studioでrosbag_recorderの状態を監視するための設定ファイルが含まれています。

## 提供される可視化

### 1. 記録ステータスインジケーター
- **トピック**: `/rosbag_recorder/recording_status`
- **表示**: 記録中（緑）/ 停止中（赤）
- **用途**: 現在の記録状態をひと目で確認

### 2. ストレージヘルスインジケーター  
- **トピック**: `/diagnostics`（ストレージ診断情報）
- **表示**: OK（緑）/ 警告（オレンジ）/ エラー（赤）
- **用途**: SSDの健康状態を監視

### 3. ディスク使用量ゲージ
- **トピック**: `/diagnostics`（ディスク使用率）
- **表示**: 0-100%のゲージ
- **用途**: ディスク容量の監視

### 4. 記録統計プロット
- **トピック**: `/diagnostics`（統計情報）
- **表示**: 
  - 記録時間（秒）
  - ファイル数
  - 総バイト数
- **用途**: 記録の進行状況をリアルタイムで確認

### 5. 詳細情報パネル
- **トピック**: `/rosbag_recorder/recording_info`
- **表示**: 現在の記録ファイルパス、継続時間、統計など
- **用途**: 詳細な記録情報の確認

### 6. 診断ステータスパネル
- **トピック**: `/diagnostics`
- **表示**: 全診断情報（recorder、storage）
- **用途**: 詳細なシステム状態の確認

## Foxglove での使用方法

### 1. レイアウトのインポート

1. Foxglove Studioを起動
2. `File` → `Import layout` を選択
3. `rosbag_recorder_dashboard.json` を選択してインポート

### 2. 手動セットアップ

以下のパネルを手動で追加・設定できます：

#### インジケーターパネル（記録状態）
```
トピック: /rosbag_recorder/recording_status
パス: .data
条件:
- == true: 背景色=#4caf50, ラベル="RECORDING"
- == false: 背景色=#f44336, ラベル="STOPPED"
```

#### インジケーターパネル（ストレージ状態）
```
トピック: /diagnostics
パス: .status[1].level
条件:
- == 0: 背景色=#4caf50, ラベル="STORAGE OK"
- == 1: 背景色=#ff9800, ラベル="STORAGE WARNING"  
- == 2: 背景色=#f44336, ラベル="STORAGE ERROR"
```

#### ゲージパネル（ディスク使用率）
```
トピック: /diagnostics
パス: .status[1].values[:]{.key=="usage_percent"}.value
範囲: 0-100
単位: %
グラデーション: 緑→黄→赤
```

#### プロットパネル（統計）
```
パス1: /diagnostics.status[0].values[:]{.key=="recording_duration_sec"}.value
パス2: /diagnostics.status[0].values[:]{.key=="file_count"}.value  
パス3: /diagnostics.status[0].values[:]{.key=="total_bytes"}.value
```

## トピック一覧

| トピック | タイプ | 説明 |
|---------|--------|------|
| `/rosbag_recorder/recording_status` | `std_msgs/Bool` | 記録状態（true/false） |
| `/rosbag_recorder/recording_info` | `std_msgs/String` | 記録詳細情報 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | システム診断情報 |

## カスタマイズ

### 色の変更
JSON設定ファイル内の `backgroundColor`, `borderColor`, `textColor` を編集

### 表示項目の追加
診断メッセージの `values` 配列から追加のキーを参照:
- `available_bytes`: 利用可能バイト数
- `total_messages`: 総メッセージ数
- `current_bag`: 現在のバッグファイルパス

### アラート設定
条件付きスタイリングを使用してアラート表示をカスタマイズ可能

## トラブルシューティング

### データが表示されない場合
1. ROS2ノードが正常に動作しているか確認
   ```bash
   ros2 topic list | grep rosbag_recorder
   ros2 topic echo /rosbag_recorder/recording_status
   ```

2. Foxgloveの接続設定を確認
   - WebSocketポート: 9090（デフォルト）
   - ROSブリッジが動作していることを確認

### リアルタイム更新されない場合
- Foxgloveの再生設定で「ライブ」モードになっているか確認
- ネットワーク遅延の可能性を考慮
