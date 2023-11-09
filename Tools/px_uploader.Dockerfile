ARG saluki_pi_fpga_version
ARG saluki_v2_fpga_version

FROM ghcr.io/tiiuae/saluki-pi-fpga:$saluki_pi_fpga_version AS SALUKI_PI
FROM ghcr.io/tiiuae/saluki-pi-fpga:$saluki_v2_fpga_version AS SALUKI_V2

FROM python:alpine3.14

COPY --from=SALUKI_PI /firmware/saluki_pi-fpga /firmware/fpga/saluki_pi
COPY --from=SALUKI_V2 /firmware/saluki_v2-fpga /firmware/fpga/saluki_v2

WORKDIR /firmware

ENTRYPOINT ["/entrypoint.sh"]

# dependency of px_uploader.py
RUN pip3 install --user pyserial

ADD px4-firmware/Tools/px_uploader.py /bin/
ADD px4-firmware/Tools/px_uploader.entrypoint /entrypoint.sh

# copy /bin/* -> /firmware/*
ADD bin/ /firmware/

# disabled as this doesnt exist in fc-only repo
# ADD px4-firmware/ssrc_config /flight_modes
