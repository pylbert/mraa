FROM ubuntu:16.04

ENV DEBIAN_FRONTEND noninteractive

# Update apt-get
RUN apt-get -y update

# Install apt-utils
RUN apt-get -y --no-install-recommends install apt-utils

# Main Build Dependencies
RUN apt-get -y install git build-essential cmake clang-3.8 g++-4.8

# Docs Build Dependencies
RUN apt-get -y install python-sphinx doxygen graphviz

# Swig Build Dependencies
RUN apt-get -y install wget libpcre3 libpcre3-dev && \
    wget http://iotdk.intel.com/misc/tr/swig-3.0.10.tar.gz && \
    tar xf swig-3.0.10.tar.gz && cd swig-3.0.10 && \
    ./configure --prefix=/usr/ && make && make install && cd ..

# Python Build Dependencies
RUN apt-get -y install python python-dev python3 python3-dev

# Java Build Dependencies
RUN apt-get -y install default-jre default-jdk

# Json Platform Build Dependencies
RUN apt-get -y install libjson0 libjson0-dev

# Node.js Build Dependencies
RUN wget -q -O - https://raw.githubusercontent.com/creationix/nvm/v0.33.1/install.sh | bash

# Set Workdir
WORKDIR /usr/src/app

# Copy sources
COPY . .

# Update Submodules
RUN git submodule update --init --recursive

# Fix line ending issue in src/doxy2swig.py and do it executable
RUN tr -d "\r" < src/doxy2swig.py > src/_doxy2swig.py && \
    mv src/_doxy2swig.py src/doxy2swig.py && \
    chmod u+x src/doxy2swig.py

# Configure Build Arguments
ARG BUILDDOC
ARG BUILDSWIG
ARG BUILDSWIGPYTHON
ARG BUILDSWIGNODE
ARG BUILDSWIGJAVA
ARG USBPLAT=OFF
ARG FIRMATA=OFF
ARG ONEWIRE=OFF
ARG JSONPLAT=OFF
ARG IMRAA=OFF
ARG FTDI4222=OFF
ARG IPK=OFF
ARG RPM=OFF
ARG ENABLEEXAMPLES=OFF
ARG INSTALLGPIOTOOL=OFF
ARG INSTALLTOOLS=OFF
ARG BUILDTESTS=OFF
ARG CC
ARG CXX
ARG NODE_VERSION

RUN env

# Configure Build Environment
ENV NVM_DIR /root/.nvm
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64/
ENV CC $CC
ENV CXX $CXX
RUN . $NVM_DIR/nvm.sh && nvm install $NODE_VERSION && nvm use $NODE_VERSION && npm install -g node-gyp

# Change Workdir to build directory
WORKDIR /usr/src/app/build

# Run cmake
RUN . $NVM_DIR/nvm.sh && cmake \
    -DSWIG_EXECUTABLE=/usr/bin/swig \
    -DSWIG_DIR:PATH=/usr/share/swig/3.0.10/ \
    -DBUILDDOC=$BUILDDOC \
    -DBUILDSWIG=$BUILDSWIG \
    -DBUILDSWIGPYTHON=$BUILDSWIGPYTHON \
    -DBUILDSWIGNODE=$BUILDSWIGNODE \
    -DBUILDSWIGJAVA=$BUILDSWIGJAVA \
    -DUSBPLAT=$USBPLAT \
    -DFIRMATA=$FIRMATA \
    -DONEWIRE=$ONEWIRE \
    -DJSONPLAT=$JSONPLAT \
    -DIMRAA=$IMRAA \
    -DFTDI4222=$FTDI4222 \
    -DIPK=$IPK \
    -DRPM=$RPM \
    -DENABLEEXAMPLES=$ENABLEEXAMPLES \
    -DINSTALLGPIOTOOL=$INSTALLGPIOTOOL \
    -DINSTALLTOOLS=$INSTALLTOOLS \
    -DBUILDTESTS=$BUILDTESTS \
    ..

CMD make
