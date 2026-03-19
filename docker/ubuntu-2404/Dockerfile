ARG BASE_IMAGE=erl/common:24.04
FROM ${BASE_IMAGE}

WORKDIR /root

ENV SPOT_VERSION=2.14.5
RUN wget http://www.lrde.epita.fr/dload/spot/spot-${SPOT_VERSION}.tar.gz && \
    tar -xf spot-${SPOT_VERSION}.tar.gz && \
    cd spot-${SPOT_VERSION} && \
    ./configure --prefix=/usr && \
    make -j`nproc` && make install
