# ros2-tools-to-use-unity-like-gazebo
[English](README.md) | 日本語
![デモ](./figs/JointStateVelocity.gif)

このリポジトリは、`ros2_control`からロボットを制御してUnityを簡単に使用できるようにする方法を示しています。
"unity_ros2_scripts"には、Unityを起動して制御するためのPythonスクリプトが含まれています。

このリポジトリの特徴は以下の通りです：
- Unity上で`ros2_control`を使用してロボットを制御する方法を示します。
- UnityとROS2 Humbleが共存できるDockerfileを提供します。
- 位置と速度制御を使用する直動および回転ジョイントをサポートしています。
- Unityから`ros2_control`にジョイント状態（位置、速度、力）を送信します。
- 希望するタイミングでURDFモデルを生成できます。
- URDF記述から剛性、減衰、および摩擦を設定できます。

## 前提条件
1. Docker
1. Unityアカウント

## Dockerコンテナの準備
1. このリポジトリをクローン
   ```
   git clone https://github.com/hijimasa/ros2-tools-to-use-unity-like-gazebo.git
   ```
2. Gitサブモジュールの初期化
   ```
   cd ros2-tools-to-use-unity-like-gazebo/
   git submodule update --init --recursive
   ```
3. dockerディレクトリに移動
   ```
   cd docker
   ```
4. Dockerイメージをビルド
   ```
   ./build-dokcer-image.bash
   ```
5. Dockerコンテナを実行
   ```
   ./run-docker-container.bash
   ```
6. UnityHubを起動
   ```
   unityhub
   ```
7. Unityにサインイン

## 使用方法

ヒント: 別のDockerターミナルを開くには、以下のコマンドを使用します
```
docker exec -it ros-humble-unity /bin/bash
```

1. Unityを起動
   ```bash
   ros2 run unity_ros2_scripts launcher
   ```

2. Unityウィンドウをクリックしてスクリプトを有効化

3. URDFをインポート
   ```
   ros2 launch unity_diffbot_sim diffbot_spawn.launch.py
   ```

4. `ros_tcp_endpoint`を起動
   ```
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

5. 地面として使用する「3D Object Plane」を追加

6. シミュレーションを実行

7. `teleop_twist_keyboard`を起動
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```