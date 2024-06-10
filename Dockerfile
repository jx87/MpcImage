ARG IMAGE_BASE="ryuichiueda/ubuntu22.04-ros2"
ARG IMAGE_TAG="ur2t"

FROM ${IMAGE_BASE}
LABEL maintainer="Jiex" email="Jiex@mail2.sysu.edu.cn"

ADD tailsitter-planning /home/tailsitter-planning
ADD acados /home/acados
ADD nmpc_setup.bash /home/nmpc_setup.bash
ADD packages.txt /home/packages.txt
ADD requirements.txt /home/requirements.txt

WORKDIR /home

RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted" | sudo tee -a /etc/apt/sources.list \
&& echo "deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted" | sudo tee -a /etc/apt/sources.list\
&& sudo apt update -y



RUN xargs -a packages.txt apt-get install -y\
&& cd acados \
&& mkdir -p build \
&& cd build \
&& cmake -DACADOS_WITH_OSQP=ON .. \
&& make install -j4 	

RUN export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
&& pip install -r requirements.txt 

RUN cd /home/tailsitter-planning \
&& source /opt/ros/humble/setup.bash \
&& colcon build 

#echo 'source /home/nmpc_setup.bash'>>/root/.bashrc \
CMD echo "...success!"
