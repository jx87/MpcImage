#! /usr/bin/bash
#source /root/nmpc_setup.bash
export PYTHONPATH=$PYTHONPATH:/home/tailsitter-planning/src/nmpc
source /home/tailsitter-planning/install/setup.bash
ACADOS_SOURCE_DIR=/home/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
export ACADOS_SOURCE_DIR=$ACADOS_ROOT
