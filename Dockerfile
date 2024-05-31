ARG IMAGE_BASE="ryuichiueda/ubuntu22.04-ros2"
ARG IMAGE_TAG="ur2t"

FROM ${IMAGE_BASE}
LABEL maintainer="Jiex" email="Jiex@mail2.sysu.edu.cn"

ADD tailsitter-planning /home/tailsitter-planning
ADD acados /home/acados
ADD nmpc_setup.bash /home/nmpc_setup.bash

WORKDIR /home

#下载acados&python interface
RUN apt-get install python3-pip -y\
&& apt-get install git \
&& git clone https://github.com/acados/acados.git \
&& cd acados \
&& mkdir -p build \
&& cd build \
&& cmake -DACADOS_WITH_OSQP=ON .. \
&& make install -j4 	

RUN export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
&& echo 'source /home/nmpc_setup.bash'>>/root/.bashrc \
&& cd /home/tailsitter-planning \
&& colcon build \

CMD echo "...success!"
