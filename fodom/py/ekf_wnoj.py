import numpy as np
import sympy as sp
import pickle
import os


from utils import spExpSO3, spLogSO3, fix_rotation, expSO3

from typing_extensions import Tuple


class EKF_WNOJ:
    """
    Custom implementation of error state kalman filter. There are two ways to do propagation.


    1. on manifold (this implementation): the state is always re-linearized around the current belief of X.
    The propagation is done as follows:
    X_new = F(X, dt)
    where F(X, dt) = X [+] self.fwd(X, g, dt)

    The jacobian for the propagation is computed as the jacobian of:
    dx_new = F(X [+] dx, dt) [-] F(X, dt)


    2. on tangent (traditional ESKF): The propagation is accumulated in tangent space until a correction.
    dx_new = dx + self.fwd(X, g, dt)
    The jacobian for the propagation is computed as the jacobian of:
    dx_new = dx + self.fwd(X, g, dt)

    The difference is that in 2. in between correction (or updates) the Gaussian belief is linearized at the same point in the manifold.
    It is only relinearized when a measurement arrives.
    In 1. the belief is constantly relinearized.

    In the traditional ESKF, when you inject a new error into the manifold state, you also
    have to transform the covariance with the reset jacobian, to express it in the new coordinates. The reset jacobian for SO3 is I
    for small angles, but in general different from it. But since in this implementation I am always injecting a new state into the manifold,
    do I need to use the reset jacobian after first order propagation? No, because I am already computing the transformation in the new coordinates
    and FOP already transforms P to the correct coordinates.

    In the update function you still need the reset jacobian.
    """

    def __init__(
        self,
        init_pose=np.eye(4, 4),
        init_stamp=0.0,
        init_vel=np.zeros((3,)),
        init_acc=np.zeros((3,)),
        init_w=np.zeros((3,)),
        init_ba=np.zeros((3,)),
        init_bg=np.zeros((3,)),
        g=np.array([0.0, 0.0, -0.01]),
        init_cov=1e-6,
        acc_std=0.01 * np.ones((3,)),
        gyro_std=0.01 * np.ones((3,)),
        compute_jacobians=False,
    ):
        self.stamp = init_stamp

        ### state
        self.p = init_pose[:3, 3]
        self.v = init_vel
        self.a = init_acc
        self.R = init_pose[:3, :3]
        self.w = init_w
        self.ba = init_ba
        self.bg = init_bg
        ###

        self.P = np.eye(21, 21) * init_cov

        self.g = g

        self.Q = np.zeros((21, 21))
        self.Q[6:9, 6:9] = np.diag(acc_std**2)
        self.Q[12:15, 12:15] = np.diag(gyro_std**2)

        cache_filename = "jacobians_wnoj.pkl"

        if compute_jacobians or not os.path.exists(cache_filename):
            state_sym = EKF_WNOJ._generate_state_sym()
            tangent_sym = EKF_WNOJ._generate_tangent_sym()
            additional_sym = EKF_WNOJ._generate_additional_sym()
            se3_sym = EKF_WNOJ._generate_se3_update_sym()

            fwd_sym = EKF_WNOJ._generate_fwd_model(state_sym, additional_sym)
            fwd_J_sym = EKF_WNOJ._generate_fwd_jacobians(
                state_sym, tangent_sym, fwd_sym
            )

            se3_update_sym = EKF_WNOJ._generate_se3_update_residual(state_sym, se3_sym)
            se3_update_J_sym = EKF_WNOJ._generate_se3_update_jacobians(
                state_sym, tangent_sym, se3_update_sym
            )
            J_reset_sym = EKF_WNOJ._generate_reset_jacobian(state_sym, tangent_sym)

            pickle.dump(
                (
                    state_sym,
                    tangent_sym,
                    additional_sym,
                    se3_sym,
                    fwd_sym,
                    fwd_J_sym,
                    se3_update_sym,
                    se3_update_J_sym,
                    J_reset_sym,
                ),
                open(cache_filename, "wb"),
            )
        else:
            (
                state_sym,
                tangent_sym,
                additional_sym,
                se3_sym,
                fwd_sym,
                fwd_J_sym,
                se3_update_sym,
                se3_update_J_sym,
                J_reset_sym,
            ) = pickle.load(open(cache_filename, "rb"))

        self.fwd = sp.lambdify(
            state_sym + additional_sym,
            EKF_WNOJ._tuple_to_vector_sym(fwd_sym),
            modules="numpy",
            cse=True,
        )
        self.fwd_J = sp.lambdify(
            state_sym + additional_sym, fwd_J_sym, modules="numpy", cse=True
        )

        self._se3_update_residual = sp.lambdify(
            state_sym + se3_sym, se3_update_sym, modules="numpy", cse=True
        )

        self.se3_update_J = sp.lambdify(
            state_sym + se3_sym, se3_update_J_sym, modules="numpy", cse=True
        )

        self.J_reset = sp.lambdify(
            state_sym + tangent_sym, J_reset_sym, modules="numpy", cse=True
        )

    def process_imu(self, imu_stamp, lin_acc, ang_vel):
        raise NotImplementedError()

    def process_pose(self, pose_stamp, pose):
        return self.pose_update(pose)

    def pose_update(self, pose):
        state = self._get_state()

        p_up = pose[:3, 3].reshape(3, 1)
        R_up = pose[:3, :3]

        r = self._se3_update_residual(*state, p_up, R_up)

        H = self.se3_update_J(*state, p_up, R_up)

        R_noise = np.eye(6) * 0.001

        S = H @ self.P @ H.T + R_noise

        K = self.P @ H.T @ np.linalg.inv(S)

        dx = (K @ r).flatten()
        self.apply_perturbation(dx)

        self.P = (np.eye(*self.P.shape) - K @ H) @ self.P

        dx_decoupled = EKF_WNOJ._vector_to_tuple_sym(dx.reshape((-1, 1)))

        J_reset = self.J_reset(*state, *dx_decoupled)

        self.P = J_reset @ self.P @ J_reset.T

        # preserve symmetric
        self.P = (self.P + self.P.T) / 2.0

    def _get_state(self):
        return (
            self.p.reshape(3, 1),
            self.v.reshape(3, 1),
            self.a.reshape(3, 1),
            self.R,
            self.w.reshape(3, 1),
            self.ba.reshape(3, 1),
            self.bg.reshape(3, 1),
        )

    def propagate(self, dt):
        state = self._get_state()

        dx = self.fwd(*state, self.g.reshape(3, 1), dt)
        J = self.fwd_J(*state, self.g.reshape(3, 1), dt)
        self.apply_perturbation(dx)

        self.P = J @ self.P @ J.T + self.Q * dt

        self.stamp += dt

    def apply_perturbation(self, dx: np.ndarray):
        dx = dx.flatten()

        dp, dv, da, dR, dw, dba, dbg = (
            dx[:3],
            dx[3:6],
            dx[6:9],
            dx[9:12],
            dx[12:15],
            dx[15:18],
            dx[18:],
        )
        self.p += dp
        self.v += dv
        self.a += da
        self.R = self.R @ expSO3(dR)
        self.R = fix_rotation(self.R)
        self.w += dw
        self.ba += dba
        self.bg += dbg

    def get_pose(self):
        pose = np.eye(4, 4)
        pose[:3, :3] = self.R
        pose[:3, 3] = self.p
        return pose

    @staticmethod
    def _generate_fwd_model(state_sym, additional_sym):
        (p, v, a, R, w, ba, bg) = state_sym
        (g, dt) = additional_sym

        a_world = R * (a - ba) + g

        perturbation_sym = (
            sp.Matrix(v * dt + 0.5 * a_world * dt * dt),
            sp.Matrix(v + a_world * dt),
            sp.zeros(3, 1),
            sp.Matrix(w - bg) * dt,
            sp.zeros(3, 1),
            sp.zeros(3, 1),
            sp.zeros(3, 1),
        )

        return perturbation_sym

    @staticmethod
    def _generate_fwd_jacobians(state_sym, tangent_sym, fwd_sym):
        X = EKF_WNOJ._symbol_tuple_to_matrix_sym(state_sym)

        # the perturbation related to belief
        dx_b = EKF_WNOJ._symbol_tuple_to_matrix_sym(tangent_sym)

        X_next = EKF_WNOJ._boxplus_sym(X, fwd_sym)

        X_next_pert = tuple(
            EKF_WNOJ._perturb(state_sym, dx_b, X_next_i) for X_next_i in X_next
        )

        # residual dynamics
        dx_next: sp.Matrix = sp.simplify(
            EKF_WNOJ._tuple_to_vector_sym(EKF_WNOJ._boxminus_sym(X_next_pert, X_next))
        )

        J = dx_next.jacobian(dx_b)
        zero_subs = {sym: sp.zeros(*sym.shape) for sym in tangent_sym}
        J = sp.simplify(J.xreplace(zero_subs))

        return J

    @staticmethod
    def _generate_se3_update_residual(state_sym, se3_update_sym):
        p, _, _, R, _, _, _ = EKF_WNOJ._symbol_tuple_to_matrix_sym(state_sym)
        p_up, R_up = se3_update_sym

        residual = sp.Matrix.vstack(
            sp.Matrix(p_up - p), spLogSO3(sp.Matrix(R.transpose() * R_up))
        )

        return residual

    @staticmethod
    def _generate_se3_update_jacobians(state_sym, tangent_sym, residual):

        dx = EKF_WNOJ._symbol_tuple_to_matrix_sym(tangent_sym)

        residual_pert = EKF_WNOJ._perturb(state_sym, dx, residual)

        J = residual_pert.jacobian(dx)
        zero_subs = {sym: sp.zeros(*sym.shape) for sym in tangent_sym}
        J = sp.simplify(J.xreplace(zero_subs))

        return J

    @staticmethod
    def _perturb(
        non_perturbed_symbols: tuple[sp.MatrixSymbol, ...],
        perturbations: tuple[sp.Matrix, ...],
        expr: sp.Matrix,
    ) -> sp.Matrix:
        perturbed_state = dict(
            zip(
                non_perturbed_symbols,
                EKF_WNOJ._boxplus_sym(non_perturbed_symbols, perturbations),
            )
        )

        return sp.simplify(expr.xreplace(perturbed_state))

    @staticmethod
    def _boxplus_sym(
        state_expr: Tuple[sp.Matrix, ...] | Tuple[sp.MatrixSymbol, ...],
        dx_decoupled: Tuple[sp.Matrix, ...],
    ) -> Tuple[sp.Matrix, ...]:

        return (
            (state_expr[0] + dx_decoupled[0]),
            (state_expr[1] + dx_decoupled[1]),
            (state_expr[2] + dx_decoupled[2]),
            (state_expr[3] * spExpSO3(dx_decoupled[3])),
            (state_expr[4] + dx_decoupled[4]),
            (state_expr[5] + dx_decoupled[5]),
            (state_expr[6] + dx_decoupled[6]),
        )

    @staticmethod
    def _boxminus_sym(
        state_to: Tuple[sp.Matrix, ...],
        state_from: Tuple[sp.Matrix, ...],
    ):
        return (
            state_to[0] - state_from[0],
            state_to[1] - state_from[1],
            state_to[2] - state_from[2],
            spLogSO3(state_from[3].transpose() * state_to[3]),
            state_to[4] - state_from[4],
            state_to[5] - state_from[5],
            state_to[6] - state_from[6],
        )

    @staticmethod
    def _generate_reset_jacobian(state_sym, tangent_sym):
        X_old = EKF_WNOJ._symbol_tuple_to_matrix_sym(state_sym)

        # actual displacement
        dx = EKF_WNOJ._symbol_tuple_to_matrix_sym(tangent_sym)

        # old parametrization
        dx_old_sym = tuple(
            sp.MatrixSymbol(f"{s.name}_old", *s.shape) for s in tangent_sym
        )
        dx_old = EKF_WNOJ._symbol_tuple_to_matrix_sym(dx_old_sym)

        # compute the mapping to the new perturbation
        dx_new = EKF_WNOJ._tuple_to_vector_sym(
            EKF_WNOJ._boxminus_sym(
                EKF_WNOJ._boxplus_sym(X_old, dx_old), EKF_WNOJ._boxplus_sym(X_old, dx)
            )
        )

        J_sym = dx_new.jacobian(dx_old_sym)

        subs_dict = dict(zip(dx_old_sym, dx))

        J_reset = sp.simplify(J_sym.xreplace(subs_dict))
        return J_reset

    @staticmethod
    def _generate_state_sym():
        p_sym = sp.MatrixSymbol("p", 3, 1)
        v_sym = sp.MatrixSymbol("v", 3, 1)
        a_sym = sp.MatrixSymbol("a", 3, 1)
        R_sym = sp.MatrixSymbol("R", 3, 3)
        w_sym = sp.MatrixSymbol("w", 3, 1)
        ba_sym = sp.MatrixSymbol("ba", 3, 1)
        bg_sym = sp.MatrixSymbol("bg", 3, 1)

        inputs = (p_sym, v_sym, a_sym, R_sym, w_sym, ba_sym, bg_sym)
        return inputs

    @staticmethod
    def _generate_tangent_sym():
        dp_sym = sp.MatrixSymbol("dp", 3, 1)
        dv_sym = sp.MatrixSymbol("dv", 3, 1)
        da_sym = sp.MatrixSymbol("da", 3, 1)
        dR_sym = sp.MatrixSymbol("dR", 3, 1)
        dw_sym = sp.MatrixSymbol("dw", 3, 1)
        dba_sym = sp.MatrixSymbol("dba", 3, 1)
        dbg_sym = sp.MatrixSymbol("dbg", 3, 1)

        inputs = (dp_sym, dv_sym, da_sym, dR_sym, dw_sym, dba_sym, dbg_sym)
        return inputs

    @staticmethod
    def _generate_additional_sym():
        # generate additional symbols for gravity and dt
        g_sym = sp.MatrixSymbol("g", 3, 1)
        dt = sp.symbols("dt")

        return (g_sym, dt)

    @staticmethod
    def _generate_se3_update_sym():
        R_up = sp.MatrixSymbol("R_up", 3, 3)
        p_up = sp.MatrixSymbol("p_up", 3, 1)

        return p_up, R_up

    @staticmethod
    def _vector_to_tuple_sym(dx):
        dx_decoupled = (
            dx[:3, 0].reshape((-1, 1)),
            dx[3:6, 0].reshape((-1, 1)),
            dx[6:9, 0].reshape((-1, 1)),
            dx[9:12, 0].reshape((-1, 1)),
            dx[12:15, 0].reshape((-1, 1)),
            dx[15:18, 0].reshape((-1, 1)),
            dx[18:, 0].reshape((-1, 1)),
        )

        return dx_decoupled

    @staticmethod
    def _tuple_to_vector_sym(dx_decoupled) -> sp.Matrix:
        dx = sp.Matrix.vstack(
            dx_decoupled[0],
            dx_decoupled[1],
            dx_decoupled[2],
            dx_decoupled[3],
            dx_decoupled[4],
            dx_decoupled[5],
            dx_decoupled[6],
        )

        return dx

    @staticmethod
    def _symbol_tuple_to_matrix_sym(syms):
        return tuple(sp.Matrix(s) for s in syms)


ekf = EKF_WNOJ(compute_jacobians=False)

for i in range(10):
    ekf.propagate(0.01)
    ekf.process_pose(None, np.eye(4, 4))
