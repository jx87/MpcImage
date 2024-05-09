from config.config import MpcConfig, ControlConfig, simulator_config


from experiments.exp_setup import simple_simulator, ref_states, ref_ts, ref_us
# from experiments import exp_setup
from utils.trajectories import get_reference_chunk

from model import quadrotorFLU, quadrotorFLU_gazebo


# msgs setting 
from interfaces.msg import Controls
from interfaces.msg import TrajectoryControl, TrajectoryPoint
from interfaces.msg import State
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


from rclpy.node import Node
import rclpy
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from utils.qos_setting import *
from utils.visualizations import *
from config.config import Simulator
from mpc_solver import MpcSolver

import threading
import time
import copy
import numpy as np


# utils times
import timeit

from utils.stat_plot import plot_density
from utils.simple_simulator import SimNode

# qos_profile = QoSProfile(
#         reliability=ReliabilityPolicy.RELIABLE,
#         durability=DurabilityPolicy.TRANSIENT_LOCAL,
#         history=HistoryPolicy.KEEP_LAST,
#         depth=1
#     )





class MpcNode(Node):
    def __init__(self):
        super().__init__('mpc_solver')

        # self.path_publisher = self.create_publisher(PoseArray, 'path', 10)
        # self.timer = self.create_timer(1.0, self.publish_odometry)
        
        self.mpc_solver = MpcSolver(quadrotorFLU_gazebo(), cython=True)
        # TODO change to TrajectoryControl
        # self.traj_sub = self.create_subscription(TrajectoryControl, "control_traj", self.dump_ctrls , reliable_qos)
        self.create_subscription(State, "cur_state", self.state_cb, keep_last_qos)
        self.create_subscription(Bool, "trigger", self.trigger_cb, keep_last_qos)

        self.controls_pub = self.create_publisher(Controls, "mpc_controls", reliable_qos)
        self.pred_traj_pub = self.create_publisher(Path, "mpc_pred_traj", keep_last_qos)

        self.hover_to_tracking = False
        # self.traj_ctrls = TrajectoryControl()
        self.controls = Controls()
        self.index = 0
        self.over_sampling = ControlConfig.over_sampling
        # ref_states, ref_timestamps, ref_us = loop_trajectory_config(LoopConfig)
        self.ref_states = ref_states
        self.ref_ts = ref_ts
        self.ref_us = ref_us
        self.current_idx = 0
        self.x0 = np.zeros((13,), dtype=np.float64)
        self.u = np.zeros((4,), dtype=np.float64)
        self.start = time.time()
        self.end = time.time()
        self.time_seq = []

    def trigger_cb(self, msg):
        self.get_logger().warn("Receiving the signal of hovering to tracking!")
        trigger:Bool = msg
        self.hover_to_tracking = True

    def run_optimization(self):
        # for i in range(5):
        self.start = time.time()
        self.mpc_solver.solve(self.x0)

        self.u = self.mpc_solver.u
        self.pub_controls()
        self.end = time.time()

        self.time_seq.append(self.end-self.start)

    def pub_predict_traj(self):
        rsts = self.mpc_solver.sim_traj()
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(rsts.shape[0]):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = rsts[i, 0]
            pose.pose.position.y = rsts[i, 1]
            pose.pose.position.z = rsts[i, 2]
            pose.pose.orientation.w = rsts[i, 3]
            pose.pose.orientation.x = rsts[i, 4]
            pose.pose.orientation.y = rsts[i, 5]
            pose.pose.orientation.z = rsts[i, 6]
            msg.poses.append(pose)
        self.pred_traj_pub.publish(msg)
        
    @property
    def opt_time(self):
        return self.mpc_solver.solve_time

    def state_cb(self, state:State):
        self.x0 = state.state
        # self.opt_thread.start()
        self.set_reference()
        self.run_optimization()
        self.pub_predict_traj()

    def set_hover_target(self, target = None):
        if target is None:
            self.mpc_solver.setRefState(xTarget=ref_states[0])
        else:
            self.mpc_solver.setRefState(xTarget=target)

    def set_reference(self):
        # if self.current_idx == self.ref_states.shape[0]:
        # self.mpc_solver.setRefState(xTarget=self.ref_states[0])
        # return 
        if self.hover_to_tracking is False :
            # self.set_hover_target(np.array([0,0,3, 0,0,0,0, 0,0,0, 0,0,0]))
            self.set_hover_target()
            self.mpc_solver
            return
        print("reference")
        x_target, u_target = get_reference_chunk(
                self.ref_states,self.ref_us, self.current_idx, self.mpc_solver.Nodes, self.over_sampling
        )
        if x_target.shape[0] == 0:
            print("Over")
            return
        self.mpc_solver.setRefTrajectory(
            xTarget=x_target,
            uTarget=u_target
        )
        self.current_idx = self.current_idx + 1

    def pub_controls(self):
        self.controls.motors_u = self.u
        # self.controls.rates_sp = self.mpc_solver.mpc_predicts[10:]
        self.controls.rates_sp = self.mpc_solver.sim_one_step()[10:]
        self.controls.total_thrust = np.sum(self.controls.motors_u)
        self.controls_pub.publish(self.controls)





def main():
    rclpy.init()
    mpc_node = MpcNode()
    vis_node = VisualizationNode()
    def func(exeas):
      exeas.spin() 
    
    if (simulator_config == Simulator.SIMPLE_SIMULATOR):
        sim_node = SimNode()
        sim_node.init_simulator(ref_states[0])
        sim_thread = SingleThreadedExecutor()
        sim_thread.add_node(sim_node)    
        th3 = threading.Thread(target=func, args=(sim_thread,)) 
        th3.start()

  


    ref_path_pub = vis_node.create_publisher(Path, "ref_path", reliable_qos)
    ## TODO change it to traj_ctrl 
    vis_node.pub_path(pos=ref_states[:,:3], quat=ref_states[:,3:7], pub=ref_path_pub)
    vis_node.pub_start_point(ref_states[0,:3], ref_states[0,3:7])
    vis_node.pub_end_point(ref_states[-1,:3], ref_states[-1,3:7])
    state_pub = vis_node.create_publisher(Odometry, "cur_state_odom", reliable_qos)
    def sub_state(msg):
        state = msg.state
        vis_node.publish_odometry(pos=state[:3], quat=state[3:7], pub=state_pub, info=None)
    vis_node.create_subscription(State, "cur_state", sub_state , keep_last_qos)


    mpc_thread = SingleThreadedExecutor()
    mpc_thread.add_node(mpc_node)
    vis_thread = SingleThreadedExecutor()
    vis_thread.add_node(vis_node)


       
    th1 = threading.Thread(target=func, args=(mpc_thread,))
    th2 = threading.Thread(target=func, args=(vis_thread,))
    th2.start()
    th1.start()

    try:
        while True:
        # 你的代码逻辑放在这里
            pass
    except KeyboardInterrupt:
        
        time = np.array(mpc_node.time_seq)
        print("***********")
        print(f"mean time: {np.mean(time)}, max time: {np.max(time)}")
        plot_density(time)






if __name__ == "__main__":
    main()