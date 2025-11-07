ARG BASE_IMAGE=erl/common:24.04
FROM ${BASE_IMAGE}

WORKDIR /root

RUN wget http://www.lrde.epita.fr/dload/spot/spot-2.14.2.tar.gz && \
    tar -xf spot-2.14.2.tar.gz && \
    cd spot-2.14.2 && \
    ./configure --prefix=/usr && \
    make -j`nproc` && make install
