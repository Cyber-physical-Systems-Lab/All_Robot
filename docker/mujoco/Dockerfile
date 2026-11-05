# MuJoCo 3.x (viewer + MJCF generation + validation) for the CPS lab robot models.
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libgl1-mesa-dri libegl1 libglfw3 libosmesa6 libxrandr2 libxinerama1 \
        libxcursor1 libxi6 && \
    rm -rf /var/lib/apt/lists/*

# same pins as envs/env_mujoco.yml
RUN pip install --no-cache-dir "mujoco>=3.2,<4" "pycollada>=0.8" numpy pyyaml

ENV CPS_ROBOTS_PATH=/workspace/All_Robot
# headless by default (validation / CI); docker/run.sh switches to glfw when a display is present
ENV MUJOCO_GL=egl
WORKDIR /workspace/All_Robot
CMD ["bash"]
