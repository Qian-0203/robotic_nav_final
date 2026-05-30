#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

CONTAINER_CMD="/bin/bash"
if [ "$1" = "--task" ]; then
    CONTAINER_CMD="/bin/bash /workspaces/yolo_bringup.sh"
    shift 1
fi

# 檢查系統架構
ARCH=$(uname -m)
OS=$(uname -s)

# 適用於 x86_64 或 macOS 上的 arm64
if [ "$ARCH" = "aarch64" ]; then
    echo "Detected architecture: arm64"
    docker run -it --rm \
        --network compose_my_bridge_network \
        $PORT_MAPPING \
        --runtime=nvidia \
        --env-file "$SCRIPT_DIR/.env" \
        -v "$SCRIPT_DIR/src:/workspaces/src" \
        -v "$SCRIPT_DIR/yolo_bringup.sh:/workspaces/yolo_bringup.sh" \
        registry.screamtrumpet.csie.ncku.edu.tw/screamlab/ros2_yolo_opencv_image:latest \
        $CONTAINER_CMD "$@"
elif [ "$ARCH" = "x86_64" ] || ([ "$ARCH" = "arm64" ] && [ "$OS" = "Darwin" ]); then
    echo "Detected architecture: amd64 or macOS arm64"
    if [ "$OS" = "Darwin" ]; then
        # macOS 版本（不使用 --gpus all）
        docker run -it --rm \
            --network compose_my_bridge_network \
            $PORT_MAPPING \
            --env-file "$SCRIPT_DIR/.env" \
            -v "$SCRIPT_DIR/src:/workspaces/src" \
            -v "$SCRIPT_DIR/screenshots:/workspaces/screenshots" \
            -v "$SCRIPT_DIR/fps_screenshots:/workspaces/fps_screenshots" \
            -v "$SCRIPT_DIR/yolo_bringup.sh:/workspaces/yolo_bringup.sh" \
            registry.screamtrumpet.csie.ncku.edu.tw/screamlab/pros_cameraapi:0.0.2 \
            $CONTAINER_CMD "$@"
    else
        if grep -q 'device:[[:space:]]*["'\'']cpu["'\'']' "$SCRIPT_DIR/src/yolo_pkg/config/yolo_params.yaml"; then
            echo "YOLO config requests CPU; running without GPU support..."
            docker run -it --rm \
                --network compose_my_bridge_network \
                $PORT_MAPPING \
                --env-file "$SCRIPT_DIR/.env" \
                -v "$SCRIPT_DIR/src:/workspaces/src" \
                -v "$SCRIPT_DIR/screenshots:/workspaces/screenshots" \
                -v "$SCRIPT_DIR/fps_screenshots:/workspaces/fps_screenshots" \
                -v "$SCRIPT_DIR/yolo_bringup.sh:/workspaces/yolo_bringup.sh" \
                registry.screamtrumpet.csie.ncku.edu.tw/screamlab/pros_cameraapi:0.0.2 \
                $CONTAINER_CMD "$@"
            exit $?
        fi

        echo "Trying to run with GPU support..."
        docker run -it --rm \
            --network compose_my_bridge_network \
            $PORT_MAPPING \
            --gpus all \
            --env-file "$SCRIPT_DIR/.env" \
            -v "$SCRIPT_DIR/src:/workspaces/src" \
            -v "$SCRIPT_DIR/screenshots:/workspaces/screenshots" \
            -v "$SCRIPT_DIR/fps_screenshots:/workspaces/fps_screenshots" \
            -v "$SCRIPT_DIR/yolo_bringup.sh:/workspaces/yolo_bringup.sh" \
            registry.screamtrumpet.csie.ncku.edu.tw/screamlab/pros_cameraapi:0.0.2 \
            $CONTAINER_CMD "$@"

        # If the GPU run fails, fall back to CPU mode.
        if [ $? -ne 0 ]; then
            echo "GPU not supported or failed, falling back to CPU mode..."
            docker run -it --rm \
                --network compose_my_bridge_network \
                $PORT_MAPPING \
                --env-file "$SCRIPT_DIR/.env" \
                -v "$SCRIPT_DIR/src:/workspaces/src" \
                -v "$SCRIPT_DIR/screenshots:/workspaces/screenshots" \
                -v "$SCRIPT_DIR/fps_screenshots:/workspaces/fps_screenshots" \
                -v "$SCRIPT_DIR/yolo_bringup.sh:/workspaces/yolo_bringup.sh" \
                registry.screamtrumpet.csie.ncku.edu.tw/screamlab/pros_cameraapi:0.0.2 \
                $CONTAINER_CMD "$@"
        fi
    fi
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
