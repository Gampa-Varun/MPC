import casadi


import Quad_mpc as Quad

import time


def traj(t, dt, N):
    # give points in the trajectory from t to t + N*dt
    # straight line trajectory, from 0,0,0 to 5,5,5 in 25 seconds
    a = 6/1953125
    b = -3/15625
    c = 2/625

    XS = []

    for i in range(N):
        
        if t<25:
            # x = a*t**5 + b*t**4 + c*t**3
            # y = a*t**5 + b*t**4 + c*t**3
            # z = a*t**5 + b*t**4 + c*t**3
            # qw = 1.0
            # qx = 0.0
            # qy = 0.0
            # qz = 0.0
            # vx = 5*a*t**4 + 4*b*t**3 + 3*c*t**2
            # vy = 5*a*t**4 + 4*b*t**3 + 3*c*t**2
            # vz = 5*a*t**4 + 4*b*t**3 + 3*c*t**2
            # wx = 0.0
            # wy = 0.0
            # wz = 0.0

            x = 0
            y = 0
            z = 2
            qw = 1.0
            qx = 0.0
            qy = 0.0
            qz = 0.0
            vx = 0
            vy = 0
            vz = 0
            wx = 0.0
            wy = 0.0
            wz = 0.0

            state_n = casadi.vertcat(x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)
        else:
            # x = 5
            # y = 5
            # z = 5
            # qw = 1.0
            # qx = 0.0
            # qy = 0.0
            # qz = 0.0
            # vx = 0
            # vy = 0
            # vz = 0
            # wx = 0.0
            # wy = 0.0
            # wz = 0.0
            x = 0
            y = 0
            z = 2
            qw = 1.0
            qx = 0.0
            qy = 0.0
            qz = 0.0
            vx = 0
            vy = 0
            vz = 0
            wx = 0.0
            wy = 0.0
            wz = 0.0

            state_n = casadi.vertcat(x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)
        t = t + dt

        XS = casadi.vertcat(XS, state_n)

    return XS
    




class MPC():
    def __init__(self):

        self.quad = Quad.Quadrotor_dynamics()

        self.T = self.quad.dt

        self.N = 10

        self.x = casadi.SX.sym('x')
        self.y = casadi.SX.sym('y')
        self.z = casadi.SX.sym('z')
        self.qw = casadi.SX.sym('qw')
        self.qx = casadi.SX.sym('qx')
        self.qy = casadi.SX.sym('qy')
        self.qz = casadi.SX.sym('qz')
        self.vx = casadi.SX.sym('vx')
        self.vy = casadi.SX.sym('vy')
        self.vz = casadi.SX.sym('vz')
        self.wx = casadi.SX.sym('wx')
        self.wy = casadi.SX.sym('wy')
        self.wz = casadi.SX.sym('wz')
        self.states = casadi.vertcat(self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz, self.vx, self.vy, self.vz, self.wx, self.wy, self.wz)
        self.n_states = self.states.size()[0]

        self.T0 = casadi.SX.sym('T0')
        self.T1 = casadi.SX.sym('T1')
        self.T2 = casadi.SX.sym('T2')
        self.T3 = casadi.SX.sym('T3')
        self.controls = casadi.vertcat(self.T0, self.T1, self.T2, self.T3)
        self.n_controls = self.controls.size()[0]

        # state = casadi.DM([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        # u = casadi.DM([0.5, 0.5, 0.5, 0.5])

        # xdot = quad.rate_of_change(state, u)

        # print(xdot)

        self.rhs = self.quad.get_next_state(self.states, self.controls)

        self.f = casadi.Function('f', [self.states, self.controls], [self.rhs])
        self.U = casadi.SX.sym('U', self.n_controls, self.N)
        self.P = casadi.SX.sym('P', self.n_states + self.n_states*self.N)

        # testing the functionself.

        # x0 = casadi.DM([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # u0 = casadi.DM([0.5, 0.5, 0.5, 0.5])
        # x_dot = f(x0, u0)

        # print(x_dot)

        self.X = casadi.SX.sym('X', self.n_states, (self.N+1))

        self.obj = 0
        self.g = []

        self.Q = casadi.DM.zeros(self.n_states, self.n_states)
        self.Q[0, 0] = 5
        self.Q[1, 1] = 5
        self.Q[2, 2] = 5
        self.Q[3, 3] = 0.5
        self.Q[4, 4] = 0.5
        self.Q[5, 5] = 0.5
        self.Q[6, 6] = 0.5
        self.Q[7, 7] = 1.0
        self.Q[8, 8] = 1.0
        self.Q[9, 9] = 1.0
        self.Q[10, 10] = 0.05
        self.Q[11, 11] = 0.05
        self.Q[12, 12] = 0.05

        self.R = casadi.DM.zeros(self.n_controls, self.n_controls)
        self.R[0, 0] = 0.5
        self.R[1, 1] = 0.5
        self.R[2, 2] = 0.5
        self.R[3, 3] = 0.5

        self.st = self.X[:, 0]
        self.g = casadi.vertcat(self.g, self.st-self.P[0:13])

        for k in range(self.N):
            self.st = self.X[:, k]
            self.con = self.U[:, k]
            self.obj = self.obj + (self.st - self.P[13*k+13:13*k+26]).T @ self.Q @ (self.st - self.P[13*k+13:13*k+26]) + self.con.T @ self.R @ self.con
            self.st_next = self.X[:, k+1]
            self.st_next_euler = self.f(self.st, self.con)
            self.g = casadi.vertcat(self.g, self.st_next-self.st_next_euler)

        


        self.OPT_variables = casadi.vertcat(self.X.reshape((self.n_states*(self.N+1), 1)), self.U.reshape((self.n_controls*self.N, 1)))

        self.nlp_prob = {'f': self.obj, 'x': self.OPT_variables, 'g': self.g, 'p': self.P}

        self.opts = {'ipopt': {'max_iter': 200, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6}, 'print_time': 0}

        self.solver = casadi.nlpsol('solver', 'ipopt', self.nlp_prob, self.opts)

        self.args = {}

        self.lbg = [0 for _ in range(self.n_states*(self.N+1))]
        self.ubg = [0 for _ in range(self.n_states*(self.N+1))]

        # constraints only on control inputs. no constraints on states min force from each rotor is 0 and max is 0.1

        self.lbx = [-casadi.inf for _ in range(self.n_states*(self.N+1) + self.n_controls*self.N)]

        for i in range(self.n_states*(self.N+1), self.n_states*(self.N+1) + self.n_controls*self.N):
            self.lbx[i] = 0

        self.ubx = [casadi.inf for _ in range(self.n_states*(self.N+1) + self.n_controls*self.N)]

        for i in range(self.n_states*(self.N+1), self.n_states*(self.N+1) + self.n_controls*self.N):
            self.ubx[i] = 0.1

        self.args['lbg'] = self.lbg
        self.args['ubg'] = self.ubg
        self.args['lbx'] = self.lbx
        self.args['ubx'] = self.ubx

    def shift(self,T, t0, x0, u, f):

        st = x0
        con = u[:, 0]
        st_next = f(st, con)
        x0 = st_next
        t0 = t0 + T

        u0 = casadi.horzcat(u[:, 1:], u[:, -1])

        return t0, x0, u0

# simulation the system

def main():

    mpc = MPC()

    t0 = 0
    x0 = casadi.DM([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    xs = traj(t0, mpc.quad.dt, mpc.N)

    sim_time = 25

    # P would be set of points

    u0 = casadi.DM.zeros((mpc.N, mpc.n_controls))
    X0 = casadi.repmat(x0, 1, mpc.N+1)

    mpc_iter = 0





    start_time = time.time()
    max_time_taken = 0.0
    while(mpc_iter < sim_time/mpc.quad.dt):
        iter_start_time = time.time()
        mpc.args['p'] = casadi.vertcat(x0, xs)
        mpc.args['x0'] = casadi.vertcat(X0.reshape((mpc.n_states*(mpc.N+1), 1)), u0.reshape((mpc.n_controls*mpc.N, 1)))
        sol = mpc.solver(**mpc.args)
        optimized_sol = sol['x']
        u0 = casadi.reshape(optimized_sol[mpc.n_states*(mpc.N+1):], mpc.n_controls, mpc.N)
        t0, x0, u0 = mpc.shift(mpc.quad.dt, t0, x0, u0, mpc.f)
        X0 = casadi.reshape(optimized_sol[:mpc.n_states*(mpc.N+1)], mpc.n_states, mpc.N+1).T
        X0 = casadi.horzcat(X0[:, 1:], X0[:, -1])
        mpc_iter += 1
        xs = traj(t0, mpc.quad.dt, mpc.N)
        iter_end_time = time.time()
        time_taken = iter_end_time - iter_start_time
        if time_taken > max_time_taken:
            max_time_taken = time_taken
        

    end_time = time.time()
    print('Time:', end_time - start_time)
    print('Time per MPC iteration:', (end_time - start_time)/(mpc_iter))
    print('Max time taken:', max_time_taken)

    print('x0:', x0)

if __name__ == '__main__':
    main()