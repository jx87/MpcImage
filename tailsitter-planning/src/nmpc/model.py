#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosModel
from casadi import MX, MX, vertcat, sin, cos
from kits import *
import numpy as np 
import casadi as cs

# TODO MODEL PARARS TO CONFIG

# def tailsitter_model() -> AcadosModel:
#     model_name = "quadrotor_model"
#     c_tau = cs.DM(0.016)
#     # J = np.array([.03, .03, .06])
#     J_data = cs.DM([0.03, 0.03, 0.06])
#     J = cs.diag(J_data)
#     dz = cs.DM(0.47/2)
#     dy = cs.DM(0.47/2)
#     # set up states & controls
#     # position [m]
#     p = MX.sym('p',3)
#     # quaternion 
#     q = MX.sym('q',4)
#     # velocity [m/s]
#     v = MX.sym('v',3)
#     # rate [rad/s]
#     r = MX.sym('r',3)
#     # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
#     u = MX.sym('u', 4)
#     # u = MX.sym('u', 4)
#     # u = uu ** 2
#     U = u[0] + u[1] + u[2] + u[3]
#     # print(type(u))
#     print(U)
#     x = vertcat(p,q,v,r)

#     g = cs.vertcat(0.0, 0.0, 9.81)
#     print(J)
#     # xdot
#     # TODO size to 继承
#     p_dot = MX.sym("p_dot",3,1)
#     q_dot = MX.sym("q_dot",4,1)
#     v_dot = MX.sym("v_dot",3,1)
#     r_dot = MX.sym("r_dot",3,1)
#     x_dot = cs.vertcat(p_dot,q_dot,v_dot,r_dot)
#     # TODO to funcs
#     tb = cs.vertcat(cs.sqrt(U), 0.0, 0.0)
#     tau_b = cs.vertcat(
#         c_tau*(-u[0]+u[1]-u[2]+u[3]),
#         dz*(-u[0]-u[1]+u[2]+u[3]),
#         dy*(-u[0]+u[1]+u[2]-u[3])
#     )



#     # algebraic variables
#     # z = None

#     # parameters
#     p = []

#     # dynamics
#     f_expl = vertcat(
#         v,
#         1 / 2 * cs.mtimes(skew_symmetric(r), q),
#         v_dot_q(tb,q) + g,
#         cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))) 
#     )
#     f_impl = x_dot - f_expl

#     model = AcadosModel()

#     model.f_impl_expr = f_impl
#     model.f_expl_expr = f_expl
#     model.x = x
#     model.xdot = x_dot
#     model.u = u
#     # model.z = z
#     model.p = p
#     model.name = model_name

#     return model



def quadrotorFLU():
    model_name = "quadrotor_flu"
    # c_tau = cs.DM(0.016)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.03, 0.03, .06])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    # dz = cs.DM(0.174)
    # dy = cs.DM(0.174)
    dx = cs.DM(0.47/2)
    dy = cs.DM(0.47/2)
    # set up states & controls
    # position [m]
    p = MX.sym('p',3)
    # quaternion 
    q = MX.sym('q',4)
    # velocity [m/s]
    v = MX.sym('v',3)
    # rate [rad/s]
    r = MX.sym('r',3)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    u = MX.sym('u', 4)
    U = u[0] + u[1] + u[2] + u[3]
    # print(type(u))
    print(U)
    x = vertcat(p,q,v,r)
    
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    print(J)
    # xdot
    # TODO size to 继承
    p_dot = MX.sym("p_dot",3,1)
    q_dot = MX.sym("q_dot",4,1)
    v_dot = MX.sym("v_dot",3,1)
    r_dot = MX.sym("r_dot",3,1)
    x_dot = cs.vertcat(p_dot,q_dot,v_dot,r_dot)
    # TODO to funcs
    # body frame
    tb = cs.vertcat(0.0, 0.0, U)

    # caused by tau_b 
    # data driven simulator configuration
    # 2 yb 3
    #      xb
    # 1    0
    tau_b = cs.veccat(
        dy*(-u[0]-u[1]+u[2]+u[3]),
        dx*(-u[0]+u[1]+u[2]-u[3]),
        c_tau*(-u[0]+u[1]-u[2]+u[3])
    )
    # tau_b = cs.vertcat(
    #     dy*(u[0]-u[1]-u[2]+u[3]),
    #     dx*(u[0]-u[1]+u[2]-u[3]),
    #     -c_tau*(u[0]+u[1]-u[2]-u[3]),
    # )

    p = []

    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
        v_dot_q(tb,q) + g,
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))) 
    )
    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model



def quadrotorFLU_gazebo():
    model_name = "quadrotor_flu_gazebo"
    # c_tau = cs.DM(0.016)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.0216666667, 0.0216666667, .04])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    # dz = cs.DM(0.174)
    # dy = cs.DM(0.174)
    dx = cs.DM(0.174)
    dy = cs.DM(0.174)
    # set up states & controls
    # position [m]
    p = MX.sym('p',3)
    # quaternion 
    q = MX.sym('q',4)
    # velocity [m/s]
    v = MX.sym('v',3)
    # rate [rad/s]
    r = MX.sym('r',3)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    u = MX.sym('u', 4)
    U = u[0] + u[1] + u[2] + u[3]
    # print(type(u))
    print(U)
    x = vertcat(p,q,v,r)
    
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    print(J)
    # xdot
    # TODO size to 继承
    p_dot = MX.sym("p_dot",3,1)
    q_dot = MX.sym("q_dot",4,1)
    v_dot = MX.sym("v_dot",3,1)
    r_dot = MX.sym("r_dot",3,1)
    x_dot = cs.vertcat(p_dot,q_dot,v_dot,r_dot)
    # TODO to funcs
    # body frame
    tb = cs.vertcat(0.0, 0.0, U)

    # caused by tau_b 
    # data driven simulator configuration
    # 2 yb 3
    #      xb
    # 1    0
    tau_b = cs.veccat(
        dy*(-u[0]-u[1]+u[2]+u[3]),
        dx*(-u[0]+u[1]+u[2]-u[3]),
        c_tau*(-u[0]+u[1]-u[2]+u[3])
    )
    # tau_b = cs.vertcat(
    #     dy*(u[0]-u[1]-u[2]+u[3]),
    #     dx*(u[0]-u[1]+u[2]-u[3]),
    #     -c_tau*(u[0]+u[1]-u[2]-u[3]),
    # )

    p = []

    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
        v_dot_q(tb,q) + g,
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))) 
    )
    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model










def quadrotorFLU_thrust_cmd():
    model_name = "quadrotor_flu_thrust_cmd"
    c_tau = cs.DM(0.1)
    J_data = np.array([0.02166666666666667, 0.02166666666666667, .04])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    # dz = cs.DM(0.174)
    # dy = cs.DM(0.174)
    dx = cs.DM(0.174)
    dy = cs.DM(0.174)
    # set up states & controls
    # position [m]
    p = MX.sym('p',3)
    # quaternion 
    q = MX.sym('q',4)
    # velocity [m/s]
    v = MX.sym('v',3)
    # rate [rad/s]
    r = MX.sym('r',3)

    T = MX.sym('T',4)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    A = cs.DM(4,4)
    # A[3,3] = 1.
    constant_up = 0.0125
    temp = -1/constant_up
    A = temp * cs.DM_eye(4)
    # A[0,4] = 1.
    # A[1,5] = 1.
    # A[2,6] = 1.
    # A[3,7] = 1.
    # A[4,0] = -1.0/constant_up
    # A[5,1] = -1.0/constant_up
    # A[6,2] = -1.0/constant_up
    # A[7,3] = -1.0/constant_up
    # print(A)
    B = cs.DM(4,4)
    B = cs.DM_eye(4)/constant_up
    print(B)
    u = MX.sym('u', 4)
    U = T[0] + T[1] + T[2] + T[3]
    # print(type(u))
    print(U)
    x = vertcat(p,q,v,r,T)
    
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    print(J)
    # xdot
    # TODO size to 继承
    p_dot = MX.sym("p_dot",3,1)
    q_dot = MX.sym("q_dot",4,1)
    v_dot = MX.sym("v_dot",3,1)
    r_dot = MX.sym("r_dot",3,1)
    T_dot = MX.sym("T_dot",4,1)
    x_dot = cs.vertcat(p_dot,q_dot,v_dot,r_dot,T_dot)
    # TODO to funcs
    # body frame
    tb = cs.vertcat(0.0, 0.0, U)
    tau_b = cs.vertcat(
        dy*(T[0]-T[1]-T[2]+T[3]),
        dx*(T[0]-T[1]+T[2]-T[3]),
        -c_tau*(T[0]+T[1]-T[2]-T[3]),
    )


    # algebraic variables
    # z = None

    # parameters
    p = []
    # print((u-T)/constant_up)
    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
        v_dot_q(tb,q) + g,
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        cs.mtimes(A, T) + cs.mtimes(B, u) 
    )
    f_impl = x_dot - f_expl
    print(f_expl)

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model



def odometry_model():
    model_name = "odometry_model"
    p = MX.sym('p',3)
    # quaternion 
    q = MX.sym('q',4)
    # velocity [m/s]
    v = MX.sym('v',3)
    # rate [rad/s]
    r = MX.sym('r',3)

    x = vertcat(p,q)
    
    # xdot
    # TODO size to 继承
    p_dot = MX.sym("p_dot",3,1)
    q_dot = MX.sym("q_dot",4,1)
    x_dot = cs.vertcat(p_dot,q_dot)
    u = cs.vertcat(v, r)
    # parameters
    p = []
    # print((u-T)/constant_up)
    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
    )
    f_impl = x_dot - f_expl
    print(f_expl)

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model


if __name__ == '__main__':
    quadrotorFLU_thrust_cmd()