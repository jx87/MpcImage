#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel, AcadosSimOpts, AcadosSim
from config.config import MpcConfig
from utils.create_ocp_or_sim import create_ocp_solver_with_model_defined, create_sim_solver_with_model_defined
from model import quadrotorFLU
import numpy as np
import scipy.linalg
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
# from px4_msgs.msg import VehicleOdometry
# import copy
# TODO it will take model as argument to generate dynamic model optimization output


class MpcSolver:
    """mpc solver 
    """

    def __init__(self, model, cython=False) -> None:
        # super().__init__('MpcSolver')

        # Acados settting
        self.__ocp: AcadosOcp = create_ocp_solver_with_model_defined(model)
        if cython:
            AcadosOcpSolver.generate(
                self.__ocp, json_file="acados_ocp_" + self.__ocp.model.name + ".json"
            )
            AcadosOcpSolver.build(
                self.__ocp.code_export_directory, with_cython=True
            )
            self.__acados_ocp_solver: AcadosOcpSolver = AcadosOcpSolver.create_cython_solver(
                "acados_ocp_" + self.__ocp.model.name + ".json")
        else:
            self.__acados_ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
                self.__ocp, json_file="acados_ocp_" + self.__ocp.model.name + ".json"
            )

        self.Nodes = MpcConfig.nodes
        self.nx = self.__ocp.model.x.size()[0]
        self.nu = self.__ocp.model.u.size()[0]
        self.ny = self.nx + self.nu

        self.__sim = create_sim_solver_with_model_defined(
            model=self.__ocp.model, Nodes= self.Nodes, T=MpcConfig.T_horizon)
        AcadosSimSolver.generate(
            self.__sim, json_file="acados_sim_" + self.__sim.model.name + ".json"
        )
        AcadosSimSolver.build(self.__sim.code_export_directory, with_cython=True)
        self.__acados_sim_solver = AcadosSimSolver.create_cython_solver('acados_sim_' + self.__sim.model.name + '.json')


        self.simX = np.ndarray((self.Nodes+1, self.nx))
        self.simU = np.ndarray((self.Nodes, self.nu))

        self.mpc_predicts = np.ndarray((self.Nodes, self.nx))
        # TODO according last time optimization result to init this time
        # Init and setting
        for stage in range(self.Nodes + 1):
            self.__acados_ocp_solver.set(stage, "x", 0.0 * np.ones(self.nx,))
        for stage in range(self.Nodes):
            self.__acados_ocp_solver.set(stage, "u", np.zeros((self.nu,)))

        # print("------")
        # ROS settting
        # self.vehilce_odometry_subscriber = self.create_subscription(
        #     VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehilce_odometry_callback, qos_profile
        # )

    def setRefState(self, xTarget=None, uTarget=None):
        """
        change ref_state for mpc

        Args:
            xTarget: 1x13D (pos, attitude, vel, rate)
            uTarget: 1x4D (T_1, T_2, T_3, T_4)
        """
        if xTarget is None:
            xTarget = [
                [0, 0, 0],  # pos_w
                [0, 1, 0, 0],  # attitude
                [0, 0, 0],  # vel_w
                [0, 0, 0]  # rate_b
            ]
            xTarget = np.concatenate([xTarget[i] for i in range(4)])
        if uTarget is None:
            uTarget = [0, 0, 0, 0]

        # refState = np.concatenate([xTarget[i] for i in range(4)])
        yref = np.concatenate([xTarget, uTarget])
        for j in range(self.Nodes):
            self.__acados_ocp_solver.set(j, "yref", yref)
        self.__acados_ocp_solver.set(self.Nodes, "yref", xTarget)

    def setRefTrajectory(self, xTarget, uTarget=None):
        """
       change ref_states(trajectory) for mpc

       Args:
           xTarget: Nx13D (pos, attitude, vel, rate)
           uTarget: Nx4D (T_1, T_2, T_3, T_4)
       """
        if uTarget is not None:
            assert xTarget.shape[0] == (uTarget.shape[0] + 1) or xTarget.shape[0] == uTarget.shape[0]
        # consindering not enough states in the chunk
        while xTarget.shape[0] < self.Nodes + 1: ## finally its shape[0] becomes self.Nodes + 1
            xTarget = np.concatenate([xTarget, np.expand_dims(xTarget[-1,:], 0)], 0)
            uTarget = np.concatenate([uTarget, np.expand_dims(uTarget[-1,:], 0)], 0)
        
        for j in range(self.Nodes):
            ref = xTarget[j,:]
            yref = np.concatenate([ref, uTarget[j,:]])
            self.__acados_ocp_solver.set(j, "yref", yref)
        self.__acados_ocp_solver.set(self.Nodes, "yref", xTarget[self.Nodes,:])


    def solve(self, x0: np.ndarray) -> int:
        """
        solve the optimization

        Args:
            xCur (np.ndarray): 13D current state to formulate the problem

        Returns:
            status: the optimization result
        """
        # check
        # if xCur.size()[0] != 13:
        #     raise Exception(
        #         f"xcur's shape is not 13D, it is {xCur.size()[0]}"
        #     )
        # normalize quat. check zero
        xCur = x0.copy()
        xCur[3:7] = xCur[3:7] / np.linalg.norm(xCur[3:7])

        self.__acados_ocp_solver.set(0, "lbx", xCur)
        self.__acados_ocp_solver.set(0, "ubx", xCur)

        status = self.__acados_ocp_solver.solve()
        # print(status)
        # status 2 is maximum iteration 
        if status not in [0,2]:
            self.__acados_ocp_solver.print_statistics()
            raise Exception(
                f"acados acados_ocp_solver returned status {status} with x current {xCur}"
            )

        # if status == 2:
        #     print(
        #         f"acados acados_ocp_solver returned status {status} with {xCur}"
        #     )

        # TODO figure out acados integrator
        for i in range(self.Nodes):
            self.simX[i, :] = self.__acados_ocp_solver.get(i, "x")
            self.simU[i, :] = self.__acados_ocp_solver.get(i, "u")
        self.simX[self.Nodes, :] = self.__acados_ocp_solver.get(
            self.Nodes, "x")
        return status

    def sim_traj(self, xCur=None):
        if xCur == None:
            xCur = self.simX[0, :]
        x0 = xCur.copy()
        for i in range(self.Nodes):
            self.mpc_predicts[i,:] = np.array(self.__acados_sim_solver.simulate(x=x0, u=self.simU[i,:]))
            x0 = self.mpc_predicts[i,:]
        return self.mpc_predicts
    
    def sim_one_step(self, xCur=None):
        if xCur == None:
            xCur = self.simX[0, :]
        x0 = xCur.copy()
        return np.array(self.__acados_sim_solver.simulate(x=x0, u=self.u))
  
    # def setHorizon(self, T):
    #     self.__acados_ocp_solver.set("")
        
    @property
    def u(self) -> np.ndarray:
        if (self.nx > 13):
            return self.simX[1, 13:17]
        else:
            return self.simU[0, :]
    
    def u_seq(self, len) -> np.ndarray:
        if (self.nx > 13):
            return self.simX[:len, 13:17]
        else:
            return self.simU[:len, :]

    # def show(self):
    #     # print("******** X")
    #     # print(self.simX)
    #     # print("U")
    #     print(self.simU[0,:])
    # def vehilce_odometry_callback(self, vehicle_odometry:VehicleOdometry):
    #   pass

    @property
    def solve_time(self):
        return self.__acados_ocp_solver.get_stats('time_tot')

    def print_residuals(self):
        print(self.__acados_ocp_solver.get_residuals())


def main():
    pass


if __name__ == '__main__':
    main()
    # try:
    #     main()
    # except Exception as e:
    #     print(e)
