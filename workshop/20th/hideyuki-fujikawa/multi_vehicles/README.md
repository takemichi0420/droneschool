# 複数機体順次制御スクリプト

このスクリプトは、複数の機体（ローバー、ボート、コプター）を順次制御します。

## 概要

**実行順序:**
1. ローバー: アーム → ミッション開始 → ミッション完了待機
2. ボート: アーム → ミッション開始 → ミッション完了待機
3. コプター: アーム → 離陸 → ミッション開始 → ミッション完了待機

前の機体がミッション完了してから、次の機体の制御が開始されます。

## 前提条件

### 1. SITL の起動

`bat/multi_vehicle_dialog.bat` または `bat/multi_vehicle.bat` を使用してSITLを起動してください。

```cmd
cd workshop/20th/hideyuki-fujikawa/bat
multi_vehicle_dialog.bat
```

これにより以下の機体が起動します：
- ローバー (instance 0, sysid 1, port 5762)
- ボート (instance 1, sysid 2, port 5772)
- コプター (instance 2, sysid 3, port 5782)

### 2. ミッションの準備

各機体に事前にミッションをアップロードしておく必要があります。

Mission Planner で以下の手順を実施：
1. 各機体に接続
2. FLIGHT PLAN タブでウェイポイントを作成
3. "Write WPs" ボタンでミッションをアップロード

または、pymavlink を使用してプログラムでミッションをアップロード：
```python
# pm50_mission_basics.py を参考に、各機体にミッションをアップロード
```

## 実行方法

### WSL/Linux から実行

```bash
cd workshop/20th/hideyuki-fujikawa/multi_vehicles
python3 sequential_control.py
```

### Windows から実行

```cmd
cd workshop\20th\hideyuki-fujikawa\multi_vehicles
python sequential_control.py
```

## 接続設定

各機体の接続設定：

| 機体タイプ | ポート | 接続文字列（デフォルト） |
|-----------|--------|-----------|
| ローバー   | 5762   | tcp:127.0.0.1:5762 |
| ボート     | 5772   | tcp:127.0.0.1:5772 |
| コプター   | 5782   | tcp:127.0.0.1:5782 |

**接続先ホストの変更:**

環境変数 `SITL_HOST` を設定することで接続先を変更できます。

```bash
# Linux/macOS
export SITL_HOST=192.168.1.100
python3 sequential_control.py

# Windows (PowerShell)
$env:SITL_HOST="192.168.1.100"
python sequential_control.py

# Windows (cmd)
set SITL_HOST=192.168.1.100
python sequential_control.py
```

## スクリプトの動作

### ローバー/ボート
1. 接続
2. GUIDEDモードに変更
3. アーム
4. AUTOモードに変更（ミッション開始）
5. ミッション完了待機

### コプター
1. 接続
2. GUIDEDモードに変更
3. アーム
4. 離陸（目標高度: 3m）
5. AUTOモードに変更（ミッション開始）
6. ミッション完了待機

## トラブルシューティング

### 接続エラー
- SITLが起動しているか確認
- ポート番号が正しいか確認
- ファイアウォール設定を確認

### ミッションが開始されない
- 各機体にミッションがアップロードされているか確認
- Mission Planner で "Read WPs" を実行してミッションを確認

### コプターが離陸しない
- GUIDEDモードに変更されているか確認
- アームが完了しているか確認
- バッテリーフェイルセーフが有効になっていないか確認（SITLではバッテリー電圧を設定）

## カスタマイズ

### 機体の追加/削除

`sequential_control.py` の `main()` 関数内の `vehicles` リストを編集：

```python
# デフォルトは127.0.0.1、環境変数SITL_HOSTで変更可能
sitl_host = os.environ.get('SITL_HOST', '127.0.0.1')
vehicles = [
    VehicleController("ローバー", f"tcp:{sitl_host}:5762", "rover"),
    VehicleController("ボート", f"tcp:{sitl_host}:5772", "boat"),
    VehicleController("コプター", f"tcp:{sitl_host}:5782", "copter"),
]
```

### 離陸高度の変更

`main()` 関数内の `takeoff()` 呼び出しを編集：

```python
vehicle.takeoff(target_altitude=5.0)  # 5メートルに変更
```

## 注意事項

- このスクリプトは学習/テスト用です
- 実機での使用前に十分なテストを実施してください
- ミッション完了の検出は、モード変更（AUTOモードから別のモードへ）で判断しています
- より確実な完了検出が必要な場合は、`wait_mission_complete()` メソッドを改良してください
