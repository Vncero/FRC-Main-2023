FROM gitpod/workspace-full-vnc

# may or may not move this to init task in .gitpod.yml depending on workspace lifecycle (TODO: test further)
RUN set -e \
    && mkdir wpilibinstaller \
    && wget -P wpilibinstaller/ https://github.com/wpilibsuite/allwpilib/releases/download/v2023.4.3/WPILib_Linux-2023.4.3.tar.gz \