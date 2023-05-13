FROM gitpod/workspace-full-vnc:latest

RUN set -e && \
    sudo apt-get update && \
    sudo apt-get install -y libgtk-3-dev libudev-dev libsoup2.4-dev libwebkit2gtk-4.0-dev && \
    sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt autoremove && \
    mkdir ~/wpilibinstaller && \
    wget -P ~/wpilibinstaller https://github.com/wpilibsuite/allwpilib/releases/download/v2023.4.3/WPILib_Linux-2023.4.3.tar.gz && \
    cd ~/wpilibinstaller && \
    tar -xvf WPILib_Linux-2023.4.3.tar.gz && \
    cd ~/ && \
    git clone https://github.com/Redrield/Conductor.git && \
    cd ~/Conductor/ && \
    export NODE_OPTIONS=--openssl-legacy-provider && \
    make setup && make release && \
    cd ~/Conductor/target/release

USER gitpod