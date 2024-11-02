FROM ubuntu:focal
SHELL ["/bin/bash", "-c"]

# Install necessary packages
RUN DEBIAN_FRONTEND="noninteractive" \
    apt-get update && \
    apt-get install -y python3 python3-distutils curl libgl1 net-tools iputils-ping && \
    rm -rf /var/lib/apt/lists/*

# Install pip
RUN python3 <(curl https://bootstrap.pypa.io/get-pip.py)

# Install parrot-olympe
RUN python3 -m pip install parrot-olympe

# Copy the application code
COPY main.py .

# Set the entry point to run the application
CMD ["bash"]