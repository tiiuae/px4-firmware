FROM alpine:3.20

ARG SALUKI_FILE_INFO_JSON=saluki_file_info.json
ARG FIRMWARE_DIRECTORY
ARG PX4_EXPORT_DIR
ENV PX4_EXPORT_DIR=$PX4_EXPORT_DIR

COPY ${SALUKI_FILE_INFO_JSON} /${SALUKI_FILE_INFO_JSON}
WORKDIR /firmware

ENTRYPOINT ["/entrypoint.sh"]

# tools needed to extract binaries from px4 files
RUN apk add pigz jq

ADD px4-firmware/Tools/px_uploader.entrypoint /entrypoint.sh

# copy /bin/* -> /firmware/*
ADD ${FIRMWARE_DIRECTORY}/ /firmware/
