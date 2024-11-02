FROM pytorch/pytorch:2.1.1-cuda12.1-cudnn8-runtime
SHELL ["/bin/bash", "-c"]

ENV APP_HOME /usr/src
ENV YOLO_HOME /usr/src/ultralytics
ENV MKL_THREADING_LAYER=GNU

ADD https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.ttf \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.Unicode.ttf \
    /root/.config/Ultralytics/


RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
    
    # install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3 python3-distutils curl libgl1 net-tools iputils-ping \
    gcc git zip unzip htop libgl1 libglib2.0-0 libstdc++6 libpython3-dev gnupg g++ libusb-1.0-0 libsm6 \
    build-essential \
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


RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    loguru==0.7.2 \
    pynput==1.7.7 \
    bbox_visualizer==0.1.0 \
    joblib==1.4.2 \
    multipledispatch
    
RUN apt upgrade --no-install-recommends -y openssl tar

WORKDIR $APP_HOME
RUN git clone https://github.com/ultralytics/ultralytics.git

WORKDIR $YOLO_HOME
RUN git checkout v8.2.1  
RUN git remote set-url origin https://github.com/ultralytics/ultralytics.git
ADD https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt $YOLO_HOME

RUN python3 -m pip install --upgrade pip wheel
RUN pip install --no-cache-dir -e ".[export]" "albumentations>=1.4.6" comet pycocotools

RUN pip install protobuf==3.20.*
RUN yolo export model=tmp/yolov8n.pt format=edgetpu imgsz=32 || yolo export model=tmp/yolov8n.pt format=edgetpu imgsz=32
RUN yolo export model=tmp/yolov8n.pt format=ncnn imgsz=32
RUN pip install --no-cache-dir "paddlepaddle>=2.6.0" x2paddle
RUN pip install --no-cache-dir numpy==1.23.5
RUN pip install rospkg
RUN rm -rf tmp

COPY ./ros_entrypoint.sh /
COPY ./container.bash /

WORKDIR /catkin_ws
RUN chmod u+x /ros_entrypoint.sh

RUN echo "source /container.bash" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]