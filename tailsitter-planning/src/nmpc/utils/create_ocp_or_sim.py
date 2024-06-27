from acados_template import AcadosOcp, AcadosSim, AcadosSimSolver, AcadosOcpSolver, AcadosModel
from config.config import MpcConfig
import numpy as np
from model import quadrotorFLU, odometry_model
import casadi as ca
import time

def create_ocp_solver_with_model_defined(model) -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # model = tailsitter_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = MpcConfig.nodes


    Q_list = np.hstack((MpcConfig.qPos, MpcConfig.qQuat, MpcConfig.qVel, MpcConfig.qAngV))
    Q_mat = 2 * np.diag(Q_list)  # [x,y,x_d,y_d,th,th_d]
    # R_mat = 2 * 5 * np.diag(MpcConfig.rU)
    W_mat= np.zeros((ny,ny))
    We_mat= np.zeros((nx,nx))
    W_mat[:13+nu,:13+nu] = np.diag(np.hstack((Q_list, MpcConfig.qU)))
    # W_mat[13+nu:,13+nu:] = np.diag(MpcConfig.rU)
    We_mat[:13,:13] = Q_mat

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    
    ny = nx + nu
    ny_e = nx

    sym_p = ocp.model.x[:3]
    # Assuming the collision is located at [3,3,2]
    # We only horizontally avoid the collision
    sym_p_h = sym_p[:2]
    colli_pos = np.array([3, 3])
    colli_r = 2
    colli_r_safe = 0.5
    
    constr_h = ca.norm_2(sym_p_h - colli_pos)
    

    
    model:AcadosModel = ocp.model
    model.con_h_expr = constr_h
    model.con_h_expr_0 = constr_h
    
    ocp.dims.nh = 1
    ocp.dims.nh_0 = 1
    ocp.dims.nsh = 0
    ocp.dims.nsh_0 = 0
    
    ocp.constraints.lh = np.array([colli_r + colli_r_safe])
    ocp.constraints.uh = np.array([1000.0])
    
    ocp.constraints.lh_0 = np.array([colli_r + colli_r_safe])
    ocp.constraints.uh_0 = np.array([1000.0])




    ocp.cost.W_e = We_mat
    ocp.cost.W = W_mat

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array(MpcConfig.lb_u)
    ocp.constraints.ubu = np.array(MpcConfig.ub_u)
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.zeros(nx)


    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    # ocp.solver_options.regularize_method = "PROJECT_REDUC_HESS"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.qp_solver_iter_max = 20
    # ocp.solver_options.qp_solver_iter_max = 20
    # ocp.solver_options.qp = 20
    # ocp.solver_options.nlp_solver_tol_comp = 1e1
    # ocp.solver_options.levenberg_marquaradt = 1e2
    # ocp.solver_options.regularize_method = 'CONVEXIFY'
    # ocp.solver_options.print_level = 1
    ocp.solver_options.levenberg_marquardt = 1e-1
    ocp.solver_options.hpipm_mode = "ROBUST"
    # sim_method_num_steps
    # set prediction horizon
    ocp.solver_options.tf = MpcConfig.T_horizon

    # 4-order ERK
    ocp.solver_options.sim_method_num_stages = 4
    # ocp.
    return ocp


def create_sim_solver_with_model_defined(model, Nodes,  T):
    sim = AcadosSim()
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 1
    sim.solver_options.T = T/Nodes
    sim.solver_options.integrator_type = 'ERK'
    sim.solver_options.sens_forw = False
    sim.model = model
    return sim

# sim = AcadosSim()
# sim.solver_options.num_stages = 4
# sim.solver_options.num_steps = 1
# sim.solver_options.T = 1
# sim.solver_options.integrator_type = 'ERK'
# sim.solver_options.sens_forw = False
# # sim.solver_options.
# sim.model = odometry_model()
# # sim.dims.nx = sim.model.
# AcadosSimSolver.generate(
#     sim, json_file="acados_sim_" + sim.model.name + ".json"
# )
# AcadosSimSolver.build(sim.code_export_directory, with_cython=True)
# sim_solver = AcadosSimSolver.create_cython_solver('acados_sim_' + sim.model.name + '.json')

# x_init = np.zeros((7,))
# x_init[3] = 1.0
# u = np.zeros((6,))
# u[3] = 90 * np.pi / 180
# # u[4] = np.sin(5*np.pi/180)
# # sim_solver.set("x", x_init)
# # sim_solver.set("u", x_init)
# start_time = time.time()
# # for i in range(1000):
# # for i in range(50):
# #     sim_solver.set("T", 0.02)
# #     rst = sim_solver.simulate(x=x_init, u = u)
# #     x_init = np.array(rst)
# #     # u = np.array(rst[3:])
# # # 记录结束时间
# # end_time = time.time()

# # 计算代码执行时间
# elapsed_time = end_time - start_time

# # ocp.constraints.constr_type = 
# print(elapsed_time)
# print(rst)

# def main():
#     ocp = create_ocp_solver_with_model_defined(quadrotorFLU())
#     acados_ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
#         ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
#     )

#     acados_ocp_solver.set()


# if __name__ == "__main__":
#     main()
