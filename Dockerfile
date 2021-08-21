FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

LABEL author="Marcelo Garcia"

            
RUN apt-get update --fix-missing && apt-get install -y \
    git \
    git-core \ 
    pkg-config \
    wget \
    vim \
    unzip \
    zip \
    zlib1g-dev \
    libpcl-dev \
    cmake


WORKDIR /app

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics









