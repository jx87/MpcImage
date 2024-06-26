""" Implementation of the Simplified Simulator and its quadrotor dynamics.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""


from math import sqrt
import numpy as np
from utils.quaternions import quaternion_to_euler, skew_symmetric, v_dot_q, unit_quat, quaternion_inverse
from config.config import SimpleSimConfig
from utils.qos_setting import keep_last_qos
from interfaces.msg import State, Controls
from rclpy.node import Node

class Quadrotor3D:

    def __init__(self):
        """
        Initialization of the 3D quadrotor class
        :param noisy: Whether noise is used in the simulation
        :type noisy: bool
        :param drag: Whether to simulate drag or not.
        :type drag: bool
        :param payload: Whether to simulate a payload force in the simulation
        :type payload: bool
        :param motor_noise: Whether non-gaussian noise is considered in the motor inputs
        :type motor_noise: bool
        """

        
        # Either 'x' or '+'. The xy configuration of the thrusters in the body plane.
        configuration = 'x'

        # Maximum thrust in Newtons of a thruster when rotating at maximum speed.
        self.max_thrust = SimpleSimConfig.quad.max_thrust

        # System state space
        self.pos = np.zeros((3,))
        self.vel = np.zeros((3,))
        self.angle = np.array([1., 0., 0., 0.])  # Quaternion format: qw, qx, qy, qz
        self.a_rate = np.zeros((3,))


        # Input constraints
        self.max_input_value = 1  # Motors at full thrust
        self.min_input_value = 0  # Motors turned off

        # Quadrotor intrinsic parameters
        self.J = SimpleSimConfig.quad.J # N m s^2 = kg m^2
        self.mass = SimpleSimConfig.quad.mass  # kg

        # Length of motor to CoG segment
        self.length = SimpleSimConfig.quad.length  # m

        # Positions of thrusters
        if configuration == '+':
            self.x_f = np.array([self.length, 0, -self.length, 0])
            self.y_f = np.array([0, self.length, 0, -self.length])
        elif configuration == 'x':
            h = np.cos(np.pi / 4) * self.length
            self.x_f = np.array([h, -h, -h, h])
            self.y_f = np.array([-h, -h, h, h])

        # For z thrust torque calculation
        self.c = SimpleSimConfig.quad.c_torque  # m   (z torque generated by each motor)
        self.z_l_tau = np.array([-self.c, self.c, -self.c, self.c])

        # Gravity vector
        self.g = np.array([[0], [0], [9.81]])  # m s^-2

        # Actuation thrusts
        self.u_noiseless = np.array([0.0, 0.0, 0.0, 0.0])
        self.u = np.array([0.0, 0.0, 0.0, 0.0])  # N

        # Drag coefficients [kg / m]
        self.rotor_drag_xy = SimpleSimConfig.Drag.linear_drag
        self.rotor_drag_z = 0.0  # No rotor drag in the z dimension
        self.rotor_drag = np.array([self.rotor_drag_xy, self.rotor_drag_xy, self.rotor_drag_z])[:, np.newaxis]
        self.aero_drag = SimpleSimConfig.Drag.quadratic_drag

        self.drag = SimpleSimConfig.SimProcess.drag
        self.noisy = SimpleSimConfig.SimProcess.noisy
        self.motor_noise = SimpleSimConfig.SimProcess.motor_noise
        self.update_dt = SimpleSimConfig.SimProcess.update_dt



    def set_state(self, *args, **kwargs):
        if len(args) != 0:
            assert len(args) == 1 and len(args[0]) == 13
            self.pos[0], self.pos[1], self.pos[2], \
            self.angle[0], self.angle[1], self.angle[2], self.angle[3], \
            self.vel[0], self.vel[1], self.vel[2], \
            self.a_rate[0], self.a_rate[1], self.a_rate[2] \
                = args[0]

        else:
            self.pos = kwargs["pos"]
            self.angle = kwargs["angle"]
            self.vel = kwargs["vel"]
            self.a_rate = kwargs["rate"]


    def get_state(self, quaternion=False, stacked=False, numpy=True):

        if quaternion and not stacked:
            return [self.pos, self.angle, self.vel, self.a_rate]
        if quaternion and stacked:
            if numpy:
                return np.array([self.pos[0], self.pos[1], self.pos[2], self.angle[0], self.angle[1], self.angle[2], self.angle[3],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]])
            return [self.pos[0], self.pos[1], self.pos[2], self.angle[0], self.angle[1], self.angle[2], self.angle[3],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]]

        angle = quaternion_to_euler(self.angle)
        if not quaternion and stacked:
            if numpy:
                return np.array([self.pos[0], self.pos[1], self.pos[2], angle[0], angle[1], angle[2],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]])
            return [self.pos[0], self.pos[1], self.pos[2], angle[0], angle[1], angle[2],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]]
        return [self.pos, angle, self.vel, self.a_rate]


    def get_control(self, noisy=False):
        if not noisy:
            return self.u_noiseless
        else:
            return self.u

    def update(self, u):
        """
        Runge-Kutta 4th order dynamics integration

        :param u: 4-dimensional vector with components between [0.0, 1.0] that represent the activation of each motor.
        :param dt: time differential
        """
        dt = self.update_dt
        # Clip inputs
        for i, u_i in enumerate(u):
            self.u_noiseless[i] = max(min(u_i, self.max_input_value), self.min_input_value)

        # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)
        if self.motor_noise:
            for i, u_i in enumerate(self.u_noiseless):
                std = 0.02 * sqrt(u_i)
                noise_u = np.random.normal(loc=0.1 * (u_i / 1.3) ** 2, scale=std)
                self.u[i] = max(min(u_i - noise_u, self.max_input_value), self.min_input_value) * self.max_thrust
        else:
            self.u = self.u_noiseless * self.max_thrust

        # Generate disturbance forces / torques
        if self.noisy:
            f_d = np.random.normal(size=(3, 1), scale=10 * dt)
            t_d = np.random.normal(size=(3, 1), scale=10 * dt)
        else:
            f_d = np.zeros((3, 1))
            t_d = np.zeros((3, 1))

        x = self.get_state(quaternion=True, stacked=False)

        # RK4 integration
        k1 = [self.f_pos(x), self.f_att(x), self.f_vel(x, self.u, f_d), self.f_rate(x, self.u, t_d)]
        x_aux = [x[i] + dt / 2 * k1[i] for i in range(4)]
        k2 = [self.f_pos(x_aux), self.f_att(x_aux), self.f_vel(x_aux, self.u, f_d), self.f_rate(x_aux, self.u, t_d)]
        x_aux = [x[i] + dt / 2 * k2[i] for i in range(4)]
        k3 = [self.f_pos(x_aux), self.f_att(x_aux), self.f_vel(x_aux, self.u, f_d), self.f_rate(x_aux, self.u, t_d)]
        x_aux = [x[i] + dt * k3[i] for i in range(4)]
        k4 = [self.f_pos(x_aux), self.f_att(x_aux), self.f_vel(x_aux, self.u, f_d), self.f_rate(x_aux, self.u, t_d)]
        x = [x[i] + dt * (1.0 / 6.0 * k1[i] + 2.0 / 6.0 * k2[i] + 2.0 / 6.0 * k3[i] + 1.0 / 6.0 * k4[i]) for i in
             range(4)]

        # Ensure unit quaternion
        x[1] = unit_quat(x[1])

        self.pos, self.angle, self.vel, self.a_rate = x

    def f_pos(self, x):
        """
        Time-derivative of the position vector
        :param x: 4-length array of input state with components: 3D pos, quaternion angle, 3D vel, 3D rate
        :return: position differential increment (vector): d[pos_x; pos_y]/dt
        """

        vel = x[2]
        return vel

    def f_att(self, x):
        """
        Time-derivative of the attitude in quaternion form
        :param x: 4-length array of input state with components: 3D pos, quaternion angle, 3D vel, 3D rate
        :return: attitude differential increment (quaternion qw, qx, qy, qz): da/dt
        """

        rate = x[3]
        angle_quaternion = x[1]

        return 1 / 2 * skew_symmetric(rate).dot(angle_quaternion)

    def f_vel(self, x, u, f_d):
        """
        Time-derivative of the velocity vector
        :param x: 4-length array of input state with components: 3D pos, quaternion angle, 3D vel, 3D rate
        :param u: control input vector (4-dimensional): [trust_motor_1, ..., thrust_motor_4]
        :param f_d: disturbance force vector (3-dimensional)
        :return: 3D velocity differential increment (vector): d[vel_x; vel_y; vel_z]/dt
        """

        a_thrust = np.array([[0], [0], [np.sum(u)]]) / self.mass

        if self.drag:
            # Transform velocity to body frame
            v_b = v_dot_q(x[2], quaternion_inverse(x[1]))[:, np.newaxis]
            # Compute aerodynamic drag acceleration in world frame
            a_drag = -self.aero_drag * v_b ** 2 * np.sign(v_b) / self.mass
            # Add rotor drag
            a_drag -= self.rotor_drag * v_b / self.mass
            # Transform drag acceleration to world frame
            a_drag = v_dot_q(a_drag, x[1])
        else:
            a_drag = np.zeros((3, 1))

        angle_quaternion = x[1]



        return np.squeeze(-self.g + a_drag + v_dot_q(a_thrust + f_d / self.mass, angle_quaternion))

    def f_rate(self, x, u, t_d):
        """
        Time-derivative of the angular rate
        :param x: 4-length array of input state with components: 3D pos, quaternion angle, 3D vel, 3D rate
        :param u: control input vector (4-dimensional): [trust_motor_1, ..., thrust_motor_4]
        :param t_d: disturbance torque (3D)
        :return: angular rate differential increment (scalar): dr/dt
        """

        rate = x[3]
        return np.array([
            1 / self.J[0] * (u.dot(self.y_f) + t_d[0] + (self.J[1] - self.J[2]) * rate[1] * rate[2]),
            1 / self.J[1] * (-u.dot(self.x_f) + t_d[1] + (self.J[2] - self.J[0]) * rate[2] * rate[0]),
            1 / self.J[2] * (u.dot(self.z_l_tau) + t_d[2] + (self.J[0] - self.J[1]) * rate[0] * rate[1])
        ]).squeeze()
    
def map_to_normalized_u(u):
    return u/SimpleSimConfig.quad.max_thrust

class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')
        # self.path_publisher = self.create_publisher(PoseArray, 'path', 10)
        # self.timer = self.create_timer(1.0, self.publish_odometry)
        self.update_dt = SimpleSimConfig.SimProcess.update_dt
        self.simulator = Quadrotor3D()
        self.controls_sub  = self.create_subscription(Controls, "mpc_controls", self.update, keep_last_qos)
        self.state_pub = self.create_publisher(State, "cur_state", keep_last_qos)
        # self.count = 0
        self.current_state = np.ndarray((13,), dtype=np.float64)

        # timer
        # self.pub_state = self.create_timer(self.update_dt, self.pub_state_cb)
        # using lock up technique , as a result, we should wait for optimiztion run over 
        # it controlled by mpc_controls
        # self.end_solve_sub = self.create_subscription(Bool, "end_solve", self., keep_last_qos)
        # self.current_state = 

    def init_simulator(self, state:np.ndarray):
        self.simulator.set_state(state)
        self.current_state = self.simulator.get_state(quaternion=True, stacked=True, numpy=True)
        self.pub_state_cb()

    def pub_state_cb(self):
        msg = State()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.state = self.current_state
        self.state_pub.publish(msg)
        
    def update(self, controls_msg:Controls):
        u = map_to_normalized_u(controls_msg.motors_u)
        
        # self.simulator
        # while self.count < 100:
        #     self.simulator.update(u)
        #     self.count = self.count + 1
            # print(self.simulator.get_state())
        self.simulator.update(u)
        # self.simulator.
        self.current_state = self.simulator.get_state(quaternion=True, stacked=True, numpy=True)
        self.pub_state_cb()
        # print(self.current_state)
        # print(self.simulator.get_state(quaternion=True,stacked=True, numpy=True))