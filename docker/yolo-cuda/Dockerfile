FROM nvidia/cuda:12.8.1-runtime-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG GID=1000

ENV TZ=Asia/Taipei \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    YOLO_CONFIG_DIR=/home/trainer/.config/Ultralytics \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    FORCE_CUDA=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    ca-certificates \
    libgl1 \
    libglib2.0-0 \
    locales \
    python3 \
    python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    if ! getent group "${GID}" >/dev/null; then \
        groupadd --gid "${GID}" trainer; \
    fi; \
    if getent passwd "${UID}" >/dev/null; then \
        EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1)"; \
        if [ "${EXISTING_USER}" != "trainer" ]; then \
            usermod --login trainer --home /home/trainer --move-home --shell /bin/bash "${EXISTING_USER}"; \
        fi; \
        usermod --gid "${GID}" trainer; \
    elif id -u trainer >/dev/null 2>&1; then \
        usermod --uid "${UID}" --gid "${GID}" --home /home/trainer --shell /bin/bash trainer; \
    else \
        useradd --uid "${UID}" --gid "${GID}" --create-home --shell /bin/bash trainer; \
    fi

WORKDIR /workspace
RUN mkdir -p /workspace /workspace/datasets /workspace/runs /workspace/weights && \
    chown -R "${UID}:${GID}" /workspace

COPY docker/yolo-cuda/requirements.txt /tmp/requirements.txt

RUN python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install \
        --index-url https://download.pytorch.org/whl/cu128 \
        torch torchvision torchaudio && \
    python3 -m pip install -r /tmp/requirements.txt

RUN mkdir -p /home/trainer/.config/Ultralytics && \
    chown -R "${UID}:${GID}" /home/trainer/.config

RUN chown -R "${UID}:${GID}" /workspace

USER trainer

CMD ["/bin/bash"]
