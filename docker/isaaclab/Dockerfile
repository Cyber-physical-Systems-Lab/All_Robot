# Isaac Sim 4.5 + Isaac Lab 2.1 for the CPS lab robot models.
# Mirrors IsaacLab/docker/Dockerfile.base (v2.1.0). Build/run through docker/run.sh.
# Host needs: NVIDIA driver >= 535 + NVIDIA Container Toolkit.
ARG ISAACSIM_VERSION=4.5.0
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION}

SHELL ["/bin/bash", "-c"]
ARG ISAACLAB_VERSION=v2.1.0
ARG ISAACLAB_RL=none
ENV ISAACSIM_ROOT_PATH=/isaac-sim \
    ISAACLAB_PATH=/workspace/isaaclab \
    CPS_ROBOTS_PATH=/workspace/All_Robot \
    ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    OMNI_KIT_ACCEPT_EULA=YES \
    LANG=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git libglib2.0-0 ncurses-term && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${ISAACLAB_VERSION} https://github.com/isaac-sim/IsaacLab.git ${ISAACLAB_PATH} && \
    ln -sf ${ISAACSIM_ROOT_PATH} ${ISAACLAB_PATH}/_isaac_sim

RUN ${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip install toml pyyaml && \
    apt-get update && \
    ${ISAACLAB_PATH}/_isaac_sim/python.sh ${ISAACLAB_PATH}/tools/install_deps.py apt ${ISAACLAB_PATH}/source && \
    rm -rf /var/lib/apt/lists/*

RUN ${ISAACLAB_PATH}/isaaclab.sh --install ${ISAACLAB_RL} && \
    (${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip uninstall -y quadprog || true)

RUN echo "alias python=${ISAACLAB_PATH}/_isaac_sim/python.sh" >> /root/.bashrc && \
    echo "alias isaaclab=${ISAACLAB_PATH}/isaaclab.sh" >> /root/.bashrc

# the repo is bind-mounted here by docker/compose.yaml
WORKDIR /workspace/All_Robot
ENTRYPOINT []
CMD ["bash"]
