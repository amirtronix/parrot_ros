FROM pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime
SHELL ["/bin/bash", "-c"]


RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
    
    # install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3 python3-distutils curl libgl1 net-tools iputils-ping \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros1-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"
    
RUN python3 <(curl https://bootstrap.pypa.io/get-pip.py)
        
        # Install parrot-olympe
RUN python3 -m pip install parrot-olympe
        
    # setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros1-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
COPY main.py .


ENTRYPOINT ["/ros_entrypoint.sh"]
RUN ["chmod", "+x", "/ros_entrypoint.sh"]

CMD ["bash"]