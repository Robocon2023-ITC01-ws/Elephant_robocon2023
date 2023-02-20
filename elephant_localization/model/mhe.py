#!/usr/bin/env python3
from model.elephant_model import *
import casadi as ca

class casadi_mhe_model():
    def __init__(self, n_states, n_control, Nmhe):
        self.elephant = ElephantModel()
        self.sampling_time = 0.1
        self.Nmhe = Nmhe
        self.n_states = n_states
        self.n_controls = n_control
        self.mea_noise = np.diag([0.0001, 0.0001, np.radians(2)])**2
        self.con_noise = np.diag([np.radians(2), np.radians(2), np.radians(2), np.radians(2)])**2

    def non_linear_mapped_function(self):
        #==== STATE ====
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        yaw = ca.SX.sym("yaw")
        states = ca.vertcat(x, y, yaw)
        #==== CONTROL ====
        w1 = ca.SX.sym("w1")
        w2 = ca.SX.sym("w2")
        w3 = ca.SX.sym("w3")
        w4 = ca.SX.sym("w4")
        controls = ca.vertcat(w1,w2,w3,w4)

        RHS = self.elephant.forward_kinematic(w1, w2, w3, w3, yaw)
        f = ca.Function('f', [states, controls], [RHS])
        return f
    
    def casadi_setup_mhe(self):
        self.f = self.non_linear_mapped_function()
        y_mhe = ca.SX.sym("y_mhe", self.n_states, self.Nmhe+1)
        h_mhe = ca.SX.sym("h_mhe", self.n_states, self.Nmhe+1)
        U_mhe = ca.SX.sym("U_mhe", self.n_controls, self.Nmhe)
        U_refmhe = ca.SX.sym("U_refmhe", self.n_controls, self.Nmhe)

		# === Cost Function ===
        cost_fn = 0
        g = []

		# === Noise matrix ===
        V = np.linalg.inv(np.sqrt(self.mea_noise))
        W = np.linalg.inv(np.sqrt(self.con_noise))
		# === Measurement model
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        theta = ca.SX.sym("theta")

        states = ca.vertcat(x,y,theta)
        rot3d_z = ca.vertcat(
            ca.horzcat(ca.cos(theta), ca.sin(theta), 0),
            ca.horzcat(-ca.sin(theta), ca.cos(theta), 0),
            ca.horzcat(0, 0, 1)
        )
        #RHS = rot3d_z.T @ states
        RHS = states
        f_mhe = ca.Function('f', [states], [RHS])
		#g = y_mhe[:,0] - h_mhe[:,0]

		# === Constraint and Sum cost function ===
        for k in range(self.Nmhe+1):
            meas_err = y_mhe[:,k] - h_mhe[:,k]
			#mea_err = y_mhe[:, k] - h_mhe[:, k]
            cost_fn = cost_fn + meas_err.T @ V @ meas_err

        for k in range(self.Nmhe):
            con_err = U_mhe[:,k] - U_refmhe[:,k]
            cost_fn = cost_fn + con_err.T @ W @ con_err

        for k in range(self.Nmhe):
            x_next = y_mhe[:,k] + self.sampling_time * self.f(y_mhe[:,k], U_mhe[:,k])
            g.append(y_mhe[:,k+1]-x_next)
			#g = ca.vertcat(g, y_mhe[:,k] - x_next)

		# Create free solution
        dec_var = ca.vertcat(ca.reshape(y_mhe, -1, 1), ca.reshape(U_mhe, -1, 1))
        par_var = ca.vertcat(ca.reshape(h_mhe, -1, 1), ca.reshape(U_refmhe, -1, 1))

        nlp_options = {
			'ipopt.max_iter': 5000,
			'ipopt.print_level': 0,
			'ipopt.acceptable_tol': 1e-6,
			'ipopt.acceptable_obj_change_tol': 1e-4,
			'print_time': 0
		}

        nlp_problem = {
			'f': cost_fn,
			'x': dec_var,
			'p': par_var,
			'g': ca.vertcat(*g)
		}

		# === Create casadi solver ====

        solver = ca.nlpsol('solver', 'ipopt', nlp_problem, nlp_options)

		# === Create boundaries === 

        lbx = ca.DM.zeros((self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe, 1))
        ubx = ca.DM.zeros((self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe, 1))
        lbg = 0.0
        ubg = 0.0

        lbx[0: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] = -np.inf
        lbx[1: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] = -np.inf
        lbx[2: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] = -np.pi

        ubx[0: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] =  np.inf
        ubx[1: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] =  np.inf
        ubx[2: self.n_states*(self.Nmhe+1)+self.n_controls*self.Nmhe: self.n_states] =  np.pi

        lbx[self.n_states*(self.Nmhe+1): ] = -30
        ubx[self.n_states*(self.Nmhe+1): ] =  30

        args = {
			'lbg': lbg,
			'ubg': ubg,
			'lbx': lbx,
			'ubx': ubx
		}

        return f_mhe, solver, args    



