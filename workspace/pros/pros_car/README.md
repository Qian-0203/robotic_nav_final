# pros_car 使用說明
## class diagram
![pros_car](https://github.com/alianlbj23/pros_car/blob/main/img/pros_car.drawio.png?raw=true)
## 🚀 環境初始化
1. 第一次使用或更新相依套件後，先建立 Jazzy 映像：
   ```bash
   docker build -f Dockerfile.jazzy -t pros_car:jazzy .
   ```
2. 執行以下指令進入環境：
   ```bash
   ./car_control.sh
   ```
   啟動腳本會自動執行 `colcon build` 並載入 `install/setup.bash`。
3. 如需在暫時容器內重新安裝相依套件：
   ```bash
   INSTALL_DEPENDENCIES=true ./car_control.sh
   ```
   一般重複啟動不需要安裝套件，因為相依套件已在 `pros_car:jazzy` 映像中。

## 🚗 車輛控制
執行以下指令來開始車輛控制：
```bash
ros2 run pros_car_py robot_control
```
執行後，畫面將會顯示控制介面。

### 🔹 車輛手動控制
| 鍵盤按鍵 | 功能描述 |
|---------|---------|
| `w` | **前進** |
| `s` | **後退** |
| `a` | **左斜走** |
| `d` | **右斜走** |
| `e` | **左自轉** |
| `r` | **右自轉** |
| `z` | **停止** |
| `q` | **回到主選單** |

## 🤖 手動機械臂控制
1. 進入機械臂控制模式後，選擇 **0~4 號關節** 來調整角度。
2. 角度調整指令：
   | 鍵盤按鍵 | 功能描述 |
   |---------|---------|
   | `i` | **增加角度** |
   | `k` | **減少角度** |
   | `q` | **回到關節選擇** |

## 📍 自動導航模式
共有 **兩種自動導航模式**：

### 1️⃣ 手動導航 (`manual_auto_nav`)
- **功能**：接收 **Foxglove** 所發送的 `/goal_pose` **座標** 來進行導航。

### 2️⃣ 目標導航 (`target_auto_nav`)
- **功能**：由 `car_controller.py` 內部自動 `publish` `/goal_pose` **座標**，進行自動導航。

📢 **注意**：在使用導航模式時，**按下 `q`** 即可立即停止車輛移動並退出導航模式。

---

# pros_car Usage Guide

## 🚀 Environment Setup
1. Build the Jazzy image the first time or after dependency changes:
   ```bash
   docker build -f Dockerfile.jazzy -t pros_car:jazzy .
   ```
2. Enter the environment by running:
   ```bash
   ./car_control.sh
   ```
   The launcher runs `colcon build` and sources `install/setup.bash` automatically.
3. To refresh dependencies inside the temporary container:
   ```bash
   INSTALL_DEPENDENCIES=true ./car_control.sh
   ```
   Normal repeated launches skip dependency installation because dependencies are baked into `pros_car:jazzy`.

## 🚗 Vehicle Control
Start vehicle control by running:
```bash
ros2 run pros_car_py robot_control
```
Once started, the control interface will appear.

### 🔹 Manual Vehicle Control
| Key | Action |
|---------|---------|
| `w` | **Move forward** |
| `s` | **Move backward** |
| `a` | **Move diagonally left** |
| `d` | **Move diagonally right** |
| `e` | **Rotate left** |
| `r` | **Rotate right** |
| `z` | **Stop** |
| `q` | **Return to the main menu** |

## 🤖 Manual Arm Control
1. Enter **joint control mode**, then select a joint (0~4) to adjust its angle.
2. Use the following keys to control the joint angles:
   | Key | Action |
   |---------|---------|
   | `i` | **Increase angle** |
   | `k` | **Decrease angle** |
   | `q` | **Return to joint selection** |

## 📍 Autonomous Navigation Modes
There are **two autonomous navigation modes**:

### 1️⃣ Manual Auto Navigation (`manual_auto_nav`)
- **Function**: Receives `/goal_pose` coordinates from **Foxglove** and navigates accordingly.

### 2️⃣ Target Auto Navigation (`target_auto_nav`)
- **Function**: `car_controller.py` internally **publishes** `/goal_pose` coordinates for automatic navigation.

📢 **Note**: Press `q` at any time to **stop the vehicle immediately** and exit navigation mode.
