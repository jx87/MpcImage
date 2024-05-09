import numpy as np
from enum import Enum

class ControlConfig:
    control_time = 0.02
    control_frequency = 1/control_time
    nodes = 50
    T = control_time * nodes
    # 注意配置over_sampling 始终为1
    over_sampling = int(T*control_frequency/nodes)
    # over_sampling = 1
     


class MpcConfig:
    """
    simple mpc configuration
    """
    nodes = ControlConfig.nodes
    T_horizon = ControlConfig.T
    qPos = np.array([1e1, 1e1, 1e1]) 
    qQuat = np.array([1e2, 1e2, 1e2, 5*1e1]) * 1e-1 
    qVel = np.array([1.0,1.0, 1.0]) * 1e-1 * 1
    qAngV = np.array([2.0,2.0, 2.0]) * 1e1
    qU = np.array([1e1, 1e1, 1e1, 1e1]) * 1e-2
    rU = np.array([1e2, 1e2, 1e2, 1e2]) * 1e-1
    # TODO keep sync with MpcConfig
    lb_u = np.array([0.5, 0.5, 0.5, 0.5]) * 0.0
    # keep sync with simulators
    ub_u = np.array([10., 10., 10., 10.]) * 1.6


class Simulator(Enum):
    SIMPLE_SIMULATOR = 0,
    GAZEBO = 1
    
simulator_config = Simulator.SIMPLE_SIMULATOR


class SimpleSimConfig:
    class quad:
        mass = 1.0
        J = np.array([0.03, 0.03, 0.06])
        length = 0.47/2
        c_torque = 0.013
        max_thrust = 16

    class Drag:
        linear_drag = 0.3 #0.3
        quadratic_drag = 0.08

    class SimProcess:
        noisy = False
        drag = True
        motor_noise = False
        update_dt = ControlConfig.control_time

class LoopTrajConfig:
    dt = ControlConfig.control_time
    radius = 10
    z = 3
    lin_acc = 0.2
    clockwise = False
    yawing = True
    v_max = 8




class PX4Interface:
    stable_odom_frq = ControlConfig.control_frequency
    use_sim_time = True

    class thrust_curve:
        a = 11.019786910197855
        b = 5.380213089802145


def main():

    # x,y = loop_trajectory(config=LoopTrajConfig)
    pass

    # simple_simulator.drag
    # print(simple_simulator.drag)
if __name__ == "__main__":
    main()