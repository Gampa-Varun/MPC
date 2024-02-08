# use mpc to control quadrotor, not using numpy library

import casadi
import quad_math_utils_casadi as utils

class Quadrotor_dynamics():
  def __init__(self):
      
      self.kf = 1.28192 * (10**(-8))
      self.km = 5.964552 * (10**(-3))
      self.Ixx = 16.571710 * (10**(-6))
      self.Iyy = 16.571710 * (10**(-6))
      self.Izz = 29.261652 * (10**(-6))
      self.I =  casadi.DM([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])
        #np.array([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])
      self.m = 0.027
      self.g = -9.81
      self.dt = 0.1
      self.L = 0.046
      self.ctau = 0


  def rate_of_change(self, state, u):

      x1 = state[7]
      x2 = state[8]
      x3 = state[9]

      qwb_dot = utils.quat_mult(state[3:7],[0.0, state[10], state[11], state[12]])/2.0

      x4 = qwb_dot[0]
      x5 = qwb_dot[1]
      x6 = qwb_dot[2]
      x7 = qwb_dot[3]

      TB12 = casadi.SX([0.0, 0.0])

      TB = casadi.vertcat(TB12, u[0]+u[1]+u[2]+u[3])



      acceleration = utils.quat_vector_rotate(state[3:7],TB)/self.m + casadi.DM([0.0, 0.0, self.g])
      
      x8 = acceleration[0]
      x9 = acceleration[1]
      x10 = acceleration[2]

      P = casadi.DM([[-self.L, -self.L, self.L, self.L], [self.L, -self.L, -self.L, self.L], [-self.ctau, self.ctau, -self.ctau, self.ctau]])

      tauB = casadi.mtimes(P,casadi.vertcat(u[0], u[1], u[2], u[3]))

      angular_acceleration = casadi.mtimes(casadi.inv(self.I),(tauB - casadi.cross(state[10:13],casadi.mtimes(self.I,state[10:13]))))

      x11 = angular_acceleration[0]
      x12 = angular_acceleration[1]
      x13 = angular_acceleration[2]

      x_dot = casadi.vertcat(x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13)

      return x_dot

  def get_next_state(self, state, u):
      k1 = self.rate_of_change(state, u)
      k2 = self.rate_of_change(state + self.dt/2.0 * k1, u)
      k3 = self.rate_of_change(state + self.dt/2.0 * k2, u)
      k4 = self.rate_of_change(state + self.dt * k3, u)

      state_next = state + self.dt/6.0*(k1 + 2*k2 + 2*k3 + k4)

      return state_next




# test

if __name__ == "__main__":

  quad = Quadrotor()
  u = casadi.DM([0.1, 0.1, 0.01, 0.01])


  state = casadi.DM([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])


  for i in range(10):
    state = quad.get_next_state(state, u)
  
  print(state)


