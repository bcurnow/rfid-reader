from python:3

ARG USER_ID
ARG GROUP_ID
ARG INPUT_GROUP_ID

# Don't attempt to set the user to the root user (uid=0) or group (gid=0)
RUN if [ ${USER_ID:-0} -eq 0 ] || [ ${GROUP_ID:-0} -eq 0 ]; then \
        groupadd rfidreader \
        && useradd -g rfidreader rfidreader \
        ;\
    else \
        groupadd -g ${GROUP_ID} rfidreader \
        && useradd -l -u ${USER_ID} -g rfidreader rfidreader \
        ;\
    fi \
    && install -d -m 0755 -o rfidreader -g rfidreader /home/rfidreader \
    && mkdir -p /etc/sudoers.d  \
    && echo "rfidreader ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/rfidreader-all-nopasswd

# Add the input group
RUN groupadd -g ${INPUT_GROUP_ID} input \
    && usermod -a -G input rfidreader

RUN apt-get update && apt-get -y install --no-install-recommends \
    vim \
    sudo \
    less \
    && rm -rf /var/lib/apt/lists/*

COPY ./docker-files/home/.* /home/rfidreader/

COPY ./requirements.txt /tmp

RUN pip install -r /tmp/requirements.txt

USER rfidreader

WORKDIR /rfid-reader
