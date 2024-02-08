#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, acos, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
import math
import casadi

import time

import casadi



def conjugate(q):

	q_conj = q
	q_conj[1] = -q[1]
	q_conj[2] = -q[2]
	q_conj[3] = -q[3]
	return q_conj

def quat_mult(q1, q2):

	q = casadi.vertcat(q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
						q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
						q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
						q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0])
	return q


def quat_vector_rotate(q1, v1):

	v1_extend = casadi.vertcat(0, v1[0], v1[1], v1[2])

	q1_conj = conjugate(q1)

	v1_rot_extend = quat_mult(quat_mult(q1,v1_extend),q1_conj)

	v1_rot = casadi.vertcat(v1_rot_extend[1], v1_rot_extend[2], v1_rot_extend[3])

	return v1_rot



class Quadrotor_dynamics():
  def __init__(self):
	  
	  self.kf_coeff1 = 2.130295*(10**(-11))
	  self.kf_coeff2 = 1.032633*(10**(-6))
	  self.kf_coeff3 = 5.484560*(10**(-4))
	  self.kf = 1.28192 * (10**(-8))
	  self.km = 5.964552 * (10**(-3))
	  self.Ixx = 16.571710 * (10**(-6))
	  self.Iyy = 16.571710 * (10**(-6))
	  self.Izz = 29.261652 * (10**(-6))
	  self.I =  casadi.DM([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])
		#np.array([[self.Ixx,0,0],[0,self.Iyy,0],[0,0,self.Izz]])
	  self.m = 0.028
	  self.g = -9.81
	  self.dt = 0.1
	  self.L = 0.046
	  self.ctau = 0


  def rate_of_change(self, state, u):

	#x_dot = f(x, u)

	  x1 = state[7]
	  x2 = state[8]
	  x3 = state[9]

	  qwb_dot = quat_mult(state[3:7],[0.0, state[10], state[11], state[12]])/2.0

	  x4 = qwb_dot[0]
	  x5 = qwb_dot[1]
	  x6 = qwb_dot[2]
	  x7 = qwb_dot[3]

	  TB12 = casadi.SX([0.0, 0.0])

	  TB = casadi.vertcat(TB12, u[0]+u[1]+u[2]+u[3])



	  acceleration = quat_vector_rotate(state[3:7],TB)/self.m + casadi.DM([0.0, 0.0, self.g])
	  
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
	# RK4 integration

	  k1 = self.rate_of_change(state, u)
	  k2 = self.rate_of_change(state + self.dt/2.0 * k1, u)
	  k3 = self.rate_of_change(state + self.dt/2.0 * k2, u)
	  k4 = self.rate_of_change(state + self.dt * k3, u)

	  state_next = state + self.dt/6.0*(k1 + 2*k2 + 2*k3 + k4)

	  return state_next





class mpc_controller():
	def __init__(self):

		self.quad = Quadrotor_dynamics()

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

		self.X = casadi.SX.sym('X', self.n_states, (self.N+1))

		self.obj = 0
		self.g = []

		self.Q = casadi.DM.zeros(self.n_states, self.n_states)
		self.Q[0, 0] = 5
		self.Q[1, 1] = 5
		self.Q[2, 2] = 5
		self.Q[3, 3] = 3
		self.Q[4, 4] = 3
		self.Q[5, 5] = 3
		self.Q[6, 6] = 3
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

		self.opts = {'ipopt': {'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6}, 'print_time': 0}

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

	def shift(self,T, x0, u, f):

		st = x0
		con = u[:, 0]
		x0 = f(st, con)

		u0 = casadi.horzcat(u[:, 1:], u[:, -1])

		return x0, u0










class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=1)
		
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		self.w1 = 0
		self.w2 = 0
		self.w3 = 0
		self.w4 = 0
		
		self.iter = 0

		self.mpc = mpc_controller()

		self.initial = True

		self.X0 = None

		self.u0 = None

		self.wait_time = 0.0

		self.wait_time_start = 0.0

		self.motor_speed = Actuators()
		

	def traj_evaluate(self):
		#evaluate the corresponding trajectories designed in Part 1to return the desired positions, velocities and accelerations

		#print("time: ", self.t)

		if self.t <5:

			xd = 0
			xd_dot = 0
			xd_ddot = 0

			yd = 0
			yd_dot = 0
			yd_ddot = 0

			zd  = 0 + 0*self.t**1 + 0*self.t**2 + 2/25 * self.t **3 + -3/125*self.t **4 + 6/3125*self.t **5
			zd_dot = 0 + 0 * ((self.t)**1) + 3*(2/25)*((self.t)**2) + 4*(-3/125)*((self.t)**3) + 5*(6/3125)*self.t**4
			zd_ddot = 0 +  6*(2/25)* ((self.t)**1) + 12*(-3/125)*(self.t**2) + 20*(6/3125)*self.t**3 

		elif self.t>=5 and self.t<20:

			xd =   0*(self.t-5)**1 + 0*(self.t-5)**2 + 2/675 * (self.t-5) **3  -1/3375*(self.t-5) **4 + 2/253125*(self.t-5) **5
			xd_dot =  6/675*(self.t-5)**2 - 4/3375*(self.t-5)**3 + 10/253125*(self.t-5)**4
			xd_ddot =  12/675*(self.t-5) - 12/3375*(self.t-5)**2 + 40/253125*(self.t-5)**3

			yd = 0
			yd_dot = 0
			yd_ddot = 0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		elif self.t>=20 and self.t<35:

			xd = 1
			xd_dot = 0
			xd_ddot = 0

			yd = 0 + 0*(self.t-20)**1 + 0*(self.t-20)**2 + 2/675 * (self.t-20) **3  -1/3375*(self.t-20) **4 + 2/253125*(self.t-20) **5
			yd_dot = 6/675*(self.t-20)**2 - 4/3375*(self.t-20)**3 + 10/253125*(self.t-20)**4
			yd_ddot = 12/675*(self.t-20) - 12/3375*(self.t-20)**2 + 40/253125*(self.t-20)**3

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		elif self.t>=35 and self.t < 50:

			xd = 1 + 0*(self.t-35)**1 + 0*(self.t-35)**2 - 2/675 * (self.t-35) **3  +1/3375*(self.t-35) **4 - 2/253125*(self.t-35) **5
			xd_dot = -6/675*(self.t-35)**2 + 4/3375*(self.t-35)**3 - 10/253125*(self.t-35)**4
			xd_ddot =  -12/675*(self.t-35) + 12/3375*(self.t-35)**2 - 40/253125*(self.t-35)**3

			yd = 1
			yd_dot = 0
			yd_ddot = 0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		elif self.t>=50 and self.t < 65:

			xd = 0
			xd_dot =0
			xd_ddot = 0

			yd = 1 + 0*(self.t-50)**1 + 0*(self.t-50)**2 - 2/675 * (self.t-50) **3  +1/3375*(self.t-50) **4 - 2/253125*(self.t-50) **5
			yd_dot =  -6/675*(self.t-50)**2 + 4/3375*(self.t-50)**3 - 10/253125*(self.t-50)**4
			yd_ddot =  -12/675*(self.t-50) + 12/3375*(self.t-50)**2 - 40/253125*(self.t-50)**3

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0

		else:

			xd = 0
			xd_dot =0
			xd_ddot = 0

			yd = 0
			yd_dot =  0
			yd_ddot =  0

			zd  = 1
			zd_dot = 0 
			zd_ddot = 0


		#print("xd: ", xd)



		return (xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot)

	def traj_evaluate_2(self):

		a = 6/1953125
		b = -3/15625
		c = 2/625

		dt = self.mpc.quad.dt

		XS = []

		t = self.t

		for i in range(self.mpc.N):
			t = t + dt
			if t<25:

				x = 0.0
				y = 0.0
				z = 1.0
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

				x = 0.0
				y = 0.0
				z = 1.0
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

			XS = casadi.vertcat(XS, state_n)

		return XS

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot, q, w_b):


		time_start = time.time()


		if self.initial:
			self.u0 = casadi.DM.zeros((self.mpc.N, self.mpc.n_controls))
			self.initial = False
			x_state = xyz[0,0]
			y_state = xyz[1,0]
			z_state = xyz[2,0]
			qw_state = q.w
			qx_state = q.x
			qy_state = q.y
			qz_state = q.z
			vx_state = xyz_dot[0,0]
			vy_state = xyz_dot[1,0]
			vz_state = xyz_dot[2,0]
			wx_state = w_b[0,0]
			wy_state = w_b[1,0]
			wz_state = w_b[2,0]
			x0 = casadi.vertcat(x_state, y_state, z_state, qw_state, qx_state, qy_state, qz_state, vx_state, vy_state, vz_state, wx_state, wy_state, wz_state)
			self.X0 = casadi.repmat(x0, 1, self.mpc.N+1)

		else:
			x_state = xyz[0,0]
			y_state = xyz[1,0]
			z_state = xyz[2,0]
			qw_state = q.w
			qx_state = q.x
			qy_state = q.y
			qz_state = q.z
			vx_state = xyz_dot[0,0]
			vy_state = xyz_dot[1,0]
			vz_state = xyz_dot[2,0]
			wx_state = w_b[0,0]
			wy_state = w_b[1,0]
			wz_state = w_b[2,0]
			x0 = casadi.vertcat(x_state, y_state, z_state, qw_state, qx_state, qy_state, qz_state, vx_state, vy_state, vz_state, wx_state, wy_state, wz_state)
			# print("x0: ", x0.shape)
			# print("X0",self.X0.shape)
			# self.X0[:,0] = x0

		

		XS = self.traj_evaluate_2()


		self.mpc.args['p'] = casadi.vertcat(x0,XS)
		self.mpc.args['x0'] = casadi.vertcat(self.X0.reshape((self.mpc.n_states*(self.mpc.N+1),1)), self.u0.reshape((self.mpc.n_controls*self.mpc.N,1)))
		sol = self.mpc.solver(**self.mpc.args)
		optimized_sol = sol['x']
		self.u0 = casadi.reshape(optimized_sol[self.mpc.n_states*(self.mpc.N+1):], self.mpc.n_controls, self.mpc.N)
		x0, self.u0 = self.mpc.shift(self.mpc.quad.dt, x0, self.u0, self.mpc.f)
		self.X0 = casadi.reshape(optimized_sol[:self.mpc.n_states*(self.mpc.N+1)], self.mpc.n_states, self.mpc.N+1).T
		self.X0 = casadi.horzcat(self.X0[:, 1:], self.X0[:, -1])

		control_input = self.u0[:,0]

		# convert to numpy array

		control_input = control_input.full().flatten()

		# control_input = np.array([0.1, 0.1, 0.1, 0.1])

		# f1,f2,f3,f4 = control_input
		# self.w1 = (-self.mpc.quad.kf_coeff2 + sqrt(self.mpc.quad.kf_coeff2**2 - 4*self.mpc.quad.kf_coeff1*(self.mpc.quad.kf_coeff3 - f1)))/(2*self.mpc.quad.kf_coeff1)
		# self.w1 = 0.04076521*self.w1 +  380.8359

		# self.w2 = (-self.mpc.quad.kf_coeff2 + sqrt(self.mpc.quad.kf_coeff2**2 - 4*self.mpc.quad.kf_coeff1*(self.mpc.quad.kf_coeff3 - f2)))/(2*self.mpc.quad.kf_coeff1)
		# self.w2 = 0.04076521*self.w2 +  380.8359

		# self.w3 = (-self.mpc.quad.kf_coeff2 + sqrt(self.mpc.quad.kf_coeff2**2 - 4*self.mpc.quad.kf_coeff1*(self.mpc.quad.kf_coeff3 - f3)))/(2*self.mpc.quad.kf_coeff1)
		# self.w3 = 0.04076521*self.w3 +  380.8359

		# self.w4 = (-self.mpc.quad.kf_coeff2 + sqrt(self.mpc.quad.kf_coeff2**2 - 4*self.mpc.quad.kf_coeff1*(self.mpc.quad.kf_coeff3 - f4)))/(2*self.mpc.quad.kf_coeff1)
		# self.w4 = 0.04076521*self.w4 +  380.8359

		self.w1, self.w2, self.w3, self.w4 = np.sqrt(control_input/self.mpc.quad.kf)

		motor_vel = np.asarray([[self.w1],[self.w2],[self.w3],[self.w4]])

		self.motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0],motor_vel[2,0], motor_vel[3,0]]
		self.motor_speed_pub.publish(self.motor_speed)

		time_end = time.time()
		print("time taken: ", time_end - time_start)
		print('x0: ', x0)






		
			

	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0

		if self.wait_time_start == 0.0:
			self.wait_time_start = self.t
			self.wait_time = self.t

		self.wait_time = self.t - self.wait_time_start

		if self.wait_time > self.mpc.quad.dt:

			self.wait_time_start = 0.0
			self.wait_time = 0.0
		
			# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
			w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
			
			v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
			
			xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])

			# print(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
			
			q = msg.pose.pose.orientation
			
			T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
			
			T[0:3, 3] = xyz[0:3, 0]
			
			R = T[0:3, 0:3]
			
			xyz_dot = np.dot(R, v_b)
			
			rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
			
			rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
			
			rpy = np.expand_dims(rpy, axis=1)
			
			# store the actual trajectory to be visualized later
			if (self.mutex_lock_on is not True):
				self.t_series.append(self.t)
				self.x_series.append(xyz[0, 0])
				self.y_series.append(xyz[1, 0])
				self.z_series.append(xyz[2, 0])
			
			# call the controller with the current states
			self.smc_control(xyz, xyz_dot, rpy, rpy_dot,q, w_b)

		else:
			if self.initial == False:
				self.motor_speed_pub.publish(self.motor_speed)

	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path
		
		with open("/home/varun_gampa_ubuntu20/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
		
			self.mutex_lock_on = True
		
			pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")