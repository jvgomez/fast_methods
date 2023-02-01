FROM ubuntu:latest
#ubuntu:roller

RUN apt update -y; exit 0
RUN apt-get install build-essential -y
RUN apt-get install nano cmake git doxygen cmake-curses-gui wget -y
ARG DEBIAN_FRONTEND=noninteractive #should be change
RUN apt-get install imagemagick libboost-all-dev cimg-dev -y

RUN git clone https://github.com/ferxxp/fast_methods_2023.git /fast_methods/

RUN mkdir /fast_methods/build
RUN cmake -S /fast_methods/ -B /fast_methods/build/
RUN make -C /fast_methods/build/
RUN cd /fast_methods/build

CMD /fast_methods/build/fm_benchmark /fast_methods/data/benchmark.cfg
