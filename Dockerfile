ARG IMAGE_BASE="ryuichiueda/ubuntu22.04-ros2"
ARG IMAGE_TAG="ur2t"

FROM ${IMAGE_BASE}
LABEL maintainer="Jiex" email="Jiex@mail2.sysu.edu.cn"

#把tailsitter-planning放在dockerfile的父目录下
ADD tailsitter-planning /home/tailsitter-planning

WORKDIR /home

#下载acados&python interface
RUN apt-get install python3-pip -y \
&& apt-get install git \
&& git clone https://github.com/acados/acados.git \
&& cd acados \
&& git submodule update --recursive --init \
&& mkdir -p build \
&& cd build \
&& cmake -DACADOS_WITH_OSQP=ON -DACADOS_WITH_OPENMP=ON .. \
&& make install -j4 \
&& pip install -e /home/acados/interfaces/acados_template \
&& echo "source /home/tailsitter-planning/install/setup.bash">>/root/.bashrc

#添加环境变量(或直接用copy改变～/.bashrc)
ENV PYTHONPATH="${PYTHONPATH}:/home/tailsitter-planning/src/nmpc"
ENV ACADOS_ROOT="/root/acados"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ACADOS_ROOT}/lib"
ENV ACADOS_SOURCE_DIR="${ACADOS_ROOT}"
#COPY bashrc /home/.bashrc

CMD echo "...success!"
