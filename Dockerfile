# Ubuntu 22.04 + ROS 2 Humble + Gazebo + PX4 v1.15 + YOLO, built with the same
# setup script used on WSL. Image is large (~20+ GB) and takes a long time to build.
#
#   docker build -t drone-inspector-sim .                         # CPU PyTorch
#   docker build --build-arg TORCH=gpu -t drone-inspector-sim .   # CUDA PyTorch
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
ARG TORCH=cpu
ARG USERNAME=drone

RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo locales tzdata lsb-release curl git git-lfs ca-certificates tmux \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash -G sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
ENV HOME=/home/${USERNAME} LANG=en_US.UTF-8 TERM=xterm-256color
WORKDIR ${HOME}

COPY --chown=${USERNAME}:${USERNAME} . ${HOME}/drone-inspector

RUN chmod +x ${HOME}/drone-inspector/scripts/*.sh \
    && ${HOME}/drone-inspector/scripts/setup_all.sh --${TORCH}

WORKDIR ${HOME}/drone-inspector
CMD ["bash", "-lc", "./scripts/run_sim.sh"]
