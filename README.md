# rabbits_bridge

[![CI](https://github.com/rabbits-robotics/rabbits_bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_bridge/actions/workflows/ci.yml)

PC（ROS 2）と機体のファームをつなぐノード。

| ノード | 役割 |
|---|---|
| `robot_bridge_node` | [rabcl](https://github.com/rabbits-robotics/rabcl) の共通プロトコル（`rabcl::protocol`）でファームとシリアル通信する。機体に依らず、違いはパラメータ YAML だけ |
| `stm32_bridge_node` | beta 用の旧プロトコル（固定長パケット）。`cmd`（`rabbits_msgs/AttractsCommand`）を購読するが、今の gamepad / game_client は `reference_command` しか出さないので、beta は実機に入っている旧コミットの構成で動かす。beta を共通プロトコルに移行したあと削除する |
| `transceiver_module_bridge_node` | ゲーム用の無線モジュール |

## robot_bridge

| 向き | トピック | 型 |
|---|---|---|
| 出力 | `joint_states` | `sensor_msgs/JointState`（名前は YAML の `joints`） |
| 出力 | `imu/data` | `sensor_msgs/Imu`（frame_id `imu_link`） |
| 出力 | `firmware_reboot` | `std_msgs/Empty`。ファームの再起動を検出したとき 1 回（下記） |
| 入力 | `reference_command` | `rabbits_msgs/ReferenceCommand` |

パラメータ（`config/<機体>.yaml`）:

| 名前 | 内容 |
|---|---|
| `device` | シリアルデバイス（例 `/dev/ttyACM0`、Jetson の UART は `/dev/ttyTHS1`） |
| `baud` | 115200 / 230400 / 460800 / 921600 / 1000000 / 2000000 |
| `joints` | 関節番号順の関節名。ファームの `robot.hpp` と同じ並びにし、名前は URDF と同じにする |
| `modes` | モード番号順のモード名（gamma: `load`, `fire`, `speed`, `chassis`, `stabilize`） |

```bash
ros2 launch rabbits_bridge robot_bridge.launch.py config:=$(ros2 pkg prefix rabbits_bridge)/share/rabbits_bridge/config/gamma.yaml
```

- ファームの `INFO` でプロトコル版数が違えばエラーログ、関節数が YAML と違えば警告を 1 回出す。
- `ReferenceCommand` は配列長が揃っていなければ捨てて警告する。`CHASSIS_VEL` → `BODY_POSE`（車高・ロール・ピッチ。常に送る）→ `JOINT_CMD` → `MODE` の順に書き、関節とモードは配列が空なら送らない。
- ファームの `SYSTEM` の `uptime_ms` を見て、前回より小さければ再起動（HP 0 の電源断からの復電など）とみなし `firmware_reboot` を 1 回出す。bridge 起動後に初めて `SYSTEM` を受けたときも 1 回出す（ファームが先に起動していた場合に PC 側の基準を合わせるため）。PC 側（`rabbits_interface`）はこれで視点目標などの基準をリセットする。

## テスト

```bash
colcon build --packages-up-to rabbits_bridge
colcon test --packages-select rabbits_bridge
colcon test-result --verbose
```

`test/test_robot_bridge.cpp` は socketpair でシリアルを置き換え、フレームを流して配信を確認し、`reference_command` を流して書かれたフレームを確認する。
