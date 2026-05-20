#!/bin/bash

IMAGE_NAME="${PROS_CAR_IMAGE:-pros_car:jazzy}"
NETWORK_NAME="${DOCKER_NETWORK:-compose_my_bridge_network}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
INSTALL_DEPENDENCIES="${INSTALL_DEPENDENCIES:-false}"
BUILD_WORKSPACE="${BUILD_WORKSPACE:-true}"
CONTAINER_COMMAND='source "/opt/ros/${ROS_DISTRO}/setup.bash"; if [ "${INSTALL_DEPENDENCIES}" = "true" ]; then apt-get update && rosdep install -y -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" && python3 -m pip install --break-system-packages -r requirements.txt; fi; if [ "${BUILD_WORKSPACE}" = "true" ]; then colcon build && source install/setup.bash; elif [ -f install/setup.bash ]; then source install/setup.bash; fi; exec bash -l'

# 1. 統一管理 -v 參數
VOLUME_ARGS="-v $(pwd)/src:/workspaces/src -v $(pwd)/launch:/workspaces/launch -v $(pwd)/requirements.txt:/workspaces/requirements.txt:ro"

ENV_FILE_ARGS=""
if [ -f .env ]; then
    ENV_FILE_ARGS="--env-file .env"
fi

if ! docker network inspect "$NETWORK_NAME" > /dev/null 2>&1; then
    docker network create --driver bridge "$NETWORK_NAME" > /dev/null
fi

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# 檢查系統架構與作業系統
ARCH=$(uname -m)
OS=$(uname -s)

# 初始化 GPU 相關變數
GPU_FLAGS=""
USE_GPU=false

# 檢查是否為 Linux 並且支援 NVIDIA GPU
if [ "$OS" = "Linux" ]; then
    if [ -f "/etc/nv_tegra_release" ]; then
        GPU_FLAGS="--runtime=nvidia"
        USE_GPU=true
    elif docker info --format '{{json .}}' | grep -q '"Runtimes".*nvidia'; then
        GPU_FLAGS="--gpus all"
        USE_GPU=true
    fi
fi

# 測試 GPU 是否可用
if [ "$USE_GPU" = true ]; then
    echo "Testing Docker run with GPU..."
    docker run --rm $GPU_FLAGS "$IMAGE_NAME" /bin/bash -c "echo GPU test" > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "GPU not supported or failed, disabling GPU flags."
        GPU_FLAGS=""
        USE_GPU=false
    fi
fi

echo "Detected OS: $OS, Architecture: $ARCH"
echo "ROS distro: $ROS_DISTRO"
echo "Docker image: $IMAGE_NAME"
echo "Docker network: $NETWORK_NAME"
echo "Install dependencies: $INSTALL_DEPENDENCIES"
echo "Build workspace: $BUILD_WORKSPACE"
echo "GPU Flags: $GPU_FLAGS"

# 設定適當的 Docker 參數
device_options=""

# 檢查設備並加入 --device 參數
if [ -e /dev/usb_front_wheel ]; then
    device_options+=" --device=/dev/usb_front_wheel"
fi
if [ -e /dev/usb_rear_wheel ]; then
    device_options+=" --device=/dev/usb_rear_wheel"
fi
if [ -e /dev/usb_robot_arm ]; then
    device_options+=" --device=/dev/usb_robot_arm"
fi

# 根據不同架構選擇適當的 Docker 圖像
if [ "$ARCH" = "aarch64" ]; then
    echo "Detected architecture: arm64"
    docker run -it --rm \
        --network "$NETWORK_NAME" \
        $PORT_MAPPING \
        $device_options \
        $GPU_FLAGS \
        $ENV_FILE_ARGS \
        -e ROS_DISTRO="$ROS_DISTRO" \
        -e INSTALL_DEPENDENCIES="$INSTALL_DEPENDENCIES" \
        -e BUILD_WORKSPACE="$BUILD_WORKSPACE" \
        -v "$(pwd)/src:/workspaces/src" \
        -v "$(pwd)/requirements.txt:/workspaces/requirements.txt:ro" \
        "$IMAGE_NAME" \
        /bin/bash -lc "$CONTAINER_COMMAND"

elif [ "$ARCH" = "x86_64" ] || ([ "$ARCH" = "arm64" ] && [ "$OS" = "Darwin" ]); then
    echo "Detected architecture: amd64 or macOS arm64"

    if [ "$OS" = "Darwin" ]; then
        echo "Running Docker on macOS (without GPU support)..."
        docker run -it --rm \
            --network "$NETWORK_NAME" \
            $PORT_MAPPING \
            $device_options \
            $ENV_FILE_ARGS \
            -e ROS_DISTRO="$ROS_DISTRO" \
            -e INSTALL_DEPENDENCIES="$INSTALL_DEPENDENCIES" \
            -e BUILD_WORKSPACE="$BUILD_WORKSPACE" \
            $VOLUME_ARGS \
            "$IMAGE_NAME" \
            /bin/bash -lc "$CONTAINER_COMMAND"
    else
        if [ "$USE_GPU" = true ]; then
            echo "Trying to run with GPU support..."
        else
            echo "Running without GPU support..."
        fi

        docker run -it --rm \
            --network "$NETWORK_NAME" \
            $PORT_MAPPING \
            $GPU_FLAGS \
            $device_options \
            $ENV_FILE_ARGS \
            -e ROS_DISTRO="$ROS_DISTRO" \
            -e INSTALL_DEPENDENCIES="$INSTALL_DEPENDENCIES" \
            -e BUILD_WORKSPACE="$BUILD_WORKSPACE" \
            $VOLUME_ARGS \
            "$IMAGE_NAME" \
            /bin/bash -lc "$CONTAINER_COMMAND"
        run_status=$?

        # 如果 GPU 啟動失敗，回退到 CPU 模式
        if [ $run_status -ne 0 ] && [ "$USE_GPU" = true ]; then
            echo "GPU not supported or failed, falling back to CPU mode..."
            docker run -it --rm \
                --network "$NETWORK_NAME" \
                $PORT_MAPPING \
                $ENV_FILE_ARGS \
                -e ROS_DISTRO="$ROS_DISTRO" \
                -e INSTALL_DEPENDENCIES="$INSTALL_DEPENDENCIES" \
                -e BUILD_WORKSPACE="$BUILD_WORKSPACE" \
                $device_options \
                $VOLUME_ARGS \
                "$IMAGE_NAME" \
                /bin/bash -lc "$CONTAINER_COMMAND"
        elif [ $run_status -ne 0 ]; then
            exit $run_status
        fi
    fi
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
