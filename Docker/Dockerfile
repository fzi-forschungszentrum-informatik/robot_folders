FROM ros:iron

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    zsh \
    stow \
    git \
    golang-go \
    curl \
    wget \
    tmux \
    python3-argcomplete \
    ros-humble-ros-base \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root

COPY ./entrypoint.sh /
COPY ./source_global.sh /root

ENTRYPOINT ["/entrypoint.sh"]
