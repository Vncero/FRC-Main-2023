FROM gitpod/workspace-full-vnc:latest

RUN sudo apt-get update && \
    sudo apt-get install -y libgtk-3-dev && \
    sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get remove -y '.*vim.*' && \
    sudo apt autoremove

USER gitpod

RUN set -e && \
    mkdir ~/wpilibinstaller && \
    wget -P ~/wpilibinstaller https://github.com/wpilibsuite/allwpilib/releases/download/v2023.4.3/WPILib_Linux-2023.4.3.tar.gz && \
    cd ~/wpilibinstaller && \
    tar -xvf WPILib_Linux-2023.4.3.tar.gz