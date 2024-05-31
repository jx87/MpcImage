#! /usr/bin/bash

if [ $ROS_VERSION == 1 ] || [ $ROS_DISTRO == 'foxy' ]; then
  echo "ROS_DISTRO is $ROS_DISTRO! DON'T SETUP ACADOS"
<<<<<<< HEAD
  export TAILSITTER_PLANNING_DIR="/home/$USER/tailsitter-planning"
=======
  export TAILSITTER_PLANNING_DIR="/home/$USER/pnc_tail-sitter"
>>>>>>> 76fbcfd8e7ca75789d42eeba1469a4ac3584d802
  export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
  export PATH=$PATH:$NMPC_DIR/scripts/
  export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"

## ROS2 setting, that is can run code without docker environment
else
  export ACADOS_SOURCE_DIR="$HOME/acados"
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$ACADOS_SOURCE_DIR/lib"

  if [ -f "$HOME/env/bin/activate" ]; then
  echo "You are in a host machine, running acados in virtual python environment."
  source $HOME/env/bin/activate
  else
  echo "You are in a container or WSL machine, running acados in intrinsic python environment."
  fi

<<<<<<< HEAD
  export TAILSITTER_PLANNING_DIR="$HOME/tailsitter-planning"
=======
  export TAILSITTER_PLANNING_DIR="$HOME/pnc_tail-sitter"
>>>>>>> 76fbcfd8e7ca75789d42eeba1469a4ac3584d802
  export NMPC_DIR="$TAILSITTER_PLANNING_DIR/src/nmpc"
  export PYTHONPATH="$PYTHONPATH:$NMPC_DIR"

  source $TAILSITTER_PLANNING_DIR/install/setup.bash
  export PX4_ROOT="$HOME/px4-v1.14.0-stable"

  alias nmpc_root='cd $NMPC_DIR'
  alias tailstart='python $TAILSITTER_PLANNING_DIR/src/nmpc/autostart.py'
  
fi
alias conn='python $NMPC_DIR/utils/px4_interfaces.py'
alias exp='python $NMPC_DIR/experiments/minco_exp.py'
alias print_metrics='python $NMPC_DIR/metrics/rmse.py'
export PATH=$PATH:$NMPC_DIR/scripts/
alias dds_serial='MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'
<<<<<<< HEAD


#todo
&& echo 'source /home/nmpc_setup.bash'>>/root/.bashrc \
&& echo 'export PYTHONPATH=/home/tailsitter-planning/src/nmpc:$PYTHONPATH'>>/root/.bashrc \
&& echo 'source /home/tailsitter-planning/install/setup.bash'>>/root/.bashrc \
&& echo 'ACADOS_ROOT=/root/acados'>>/root/.bashrc \
&& echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_ROOT/lib'>>/root/.bashrc \
&& echo 'export ACADOS_SOURCE_DIR=$ACADOS_ROOT'>>/root/.bashrc \
=======
>>>>>>> 76fbcfd8e7ca75789d42eeba1469a4ac3584d802