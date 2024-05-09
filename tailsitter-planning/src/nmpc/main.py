#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model import tailsitter_model
import numpy as np
import scipy.linalg
import pandas as pd
# from utils import plot_robot

# X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Intitalize the states [x,y,v,th,th_d]
X0 = np.zeros([13,1]).reshape(13,)
X0[0] = 0
X0[3] = 1

init_q = np.array([0.923879532511287,0.,0.38268343236509,0.])
ref_q = np.array([0.707106781186547,0,0.707106781186547,0])
ref_q = np.array([0.0,1.0,0.0,0.0])

zeroIndex = np.full(X0.shape, False)

posIndex = zeroIndex.copy()
posIndex[0:3] = 1

attIndex = zeroIndex.copy()
attIndex[3:7] = 1

velIndex = zeroIndex.copy()
velIndex[7:10] = 1

rateIndex = zeroIndex.copy()
rateIndex[10:] = 1
X0[attIndex] = init_q

# nmpc setting
Nodes = 20  # Define the number of discretization steps
T_horizon = 1.0  # Define the prediction horizon
F_max = 10  # Define the max force allowed

# posIndex = 0:3

# F_min = -10
# T_max = 1.0
# T_max = -1.0


def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = tailsitter_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = Nodes
    # ocp.dims.nbu = 4
    # set cost
    Q_mat = 2 * np.diag([1e2, 1e2, 1e2   , 1e2, 1e2,1e2, 1e2     ,1e1,1e1, 1e1   ,1.0,1.0, 1.0,])  # [x,y,x_d,y_d,th,th_d]
    R_mat = 2 * 5 * np.diag([1e-1, 1e-2,1e-2,1e-2])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([0,0,0,0])
    ocp.constraints.ubu = np.array([+F_max,F_max,F_max,F_max])
    # ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    # temp = np.eye(u)
    # temp[0,1]
    ocp.constraints.Jbu = np.eye(nu)
    print("******")
    print(ocp.dims.nbu)
    ocp.constraints.x0 = X0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 400
    ocp.solver_options.print_level = 1
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp





def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    
    # Aca
    # prepare simulation
    Nsim = 100
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    ny = nx + nu
    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    # TODO according last time optimization result to init this time
    # initialize solver
    for stage in range(Nodes + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(xcurrent.shape))
    for stage in range(Nodes):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    for i in range(Nsim):

        # set initial state constraint
        # 初始shootin
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)
        # update yref
        for j in range(Nodes):
            yref = np.zeros([ny,1]).reshape(ny,)
            yref[0] = 1
            yref[np.pad(attIndex,(0,4),'constant',constant_values=0)] = ref_q
            # print(yref)
            # print(yref)
            # print(yref.shape)
            acados_ocp_solver.set(j, "yref", yref)
            # acados_ocp_solver.acados_ocp.
            # print(acados_ocp_solver.get(j, "x"))
            # acados_ocp_solver.get()
        yref_N = np.zeros([nx,1]).reshape(nx,)
        yref_N[0] = 1
        yref_N[attIndex] = ref_q
        acados_ocp_solver.set(Nodes, "yref", yref_N)

        # solve ocp
        status = acados_ocp_solver.solve()
        # acados_ocp_solver.options_set('print_level', 1)
        # print(self.solve)
        # acados_ocp_solver.print_statistics()
# exception plot
        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            # plot_robot(
            #     np.linspace(0, T_horizon / Nodes * i, i + 1),
            #     F_max,
            #     simU[:i, :],
            #     simX[: i + 1, :],
            # )
            raise Exception(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        if status == 2:
            print(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )
        simU[i, :] = acados_ocp_solver.get(0, "u")

        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i, :])

        status = acados_integrator.solve()
        if status != 0:
            raise Exception(
                f"acados integrator returned status {status} in closed loop instance {i}"
            )

        # update state
        xcurrent = acados_integrator.get("x")
        simX[i + 1, :] = xcurrent

    # plot results
    # print(simX)
    df = pd.DataFrame(
        {
            'u0': simU[:,0],
            'u1': simU[:,1],
            'u2': simU[:,2],
            'u3': simU[:,3],
        }
    )
    # df['state'] = simX
    df.head()
    df.to_csv('end.csv',mode='w', index=False, header=True)

    ds = pd.DataFrame(
        {
            'pos':simX[:,posIndex].tolist(),
            'att':simX[:,attIndex].tolist(),
            'vel':simX[:,velIndex].tolist(),
            'rate':simX[:,rateIndex].tolist()
        }
    )
    ds.to_csv("pre_state.csv", mode='w',index=False,header=True)
    # print(simX[:,rateIndex])
    # plot_robot(
    #     np.linspace(0, T_horizon / Nodes * Nsim, Nsim + 1), [F_max, None], simU, simX
    # )


if __name__ == "__main__":
    closed_loop_simulation()