# Vayu stack build environment — reproducible builds on Linux, macOS and
# Windows (via Docker Desktop / WSL2). The image carries only the TOOLCHAINS;
# the source is bind-mounted at run time (see docker-compose.yml), so edits on
# the host are built immediately and outputs land back in the working tree.
#
# Covers all three build targets:
#   * firmware  — arm-none-eabi cross-compile (STM32F4)         -> build-docker/
#   * SITL      — host unit tests / sanitizers                  -> build_sitl-docker/
#   * Navigator — Qt6 ground-control station + in-app simulator -> navigator/build-docker/
#
# Flashing is intentionally NOT here: USB/SWD passthrough isn't portable across
# Docker Desktop backends. Build in the container, flash from the host (see
# tools/docker/README.md).
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
# Match the host user so files written into the bind mount aren't root-owned.
# Override on Linux if your UID differs: --build-arg USER_UID=$(id -u) ...
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
      # --- common build tooling ---
      build-essential cmake ninja-build git pkg-config ca-certificates \
      python3 python3-pip \
      # --- firmware cross-toolchain (arm-none-eabi, STM32F4) ---
      gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
      # --- Navigator (Qt6 GCS) + airframe-mesh import ---
      qt6-base-dev qt6-base-dev-tools qt6-serialport-dev \
      libgl1-mesa-dev libassimp-dev \
      # --- static analysis / coverage (CI parity, optional) ---
      cppcheck clang-tidy gcovr \
    && rm -rf /var/lib/apt/lists/*

# kconfiglib drives navhal.config at firmware configure time (matches CI).
RUN pip3 install --no-cache-dir --break-system-packages kconfiglib

# Ubuntu 24.04 ships a default 'ubuntu' user at UID 1000; drop it, then create
# a 'builder' user at the requested UID/GID for clean bind-mount ownership.
RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd -g ${USER_GID} builder 2>/dev/null || true \
 && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash builder

USER builder
WORKDIR /workspace
CMD ["bash"]
