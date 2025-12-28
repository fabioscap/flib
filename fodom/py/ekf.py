import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_scipy
import csv
import sympy as sp
import os, pickle

from typing_extensions import List


class EKF:
    def __init__(
        self,
        init_pose,
        init_stamp,
        init_vel=np.zeros(
            3,
        ),
        g=np.array([0.0, 0.0, -9.81]),
        init_ba=np.zeros((3,)),
        init_bg=np.zeros((3,)),
        update_t_sigma=0.05,
        update_r_sigma=10000,
        acc_std=0.01,
        gyro_std=0.08,
        bacc_std=0.0001,
        bgyro_std=0.0002,
        init_cov=1e-6,
        acc_sat=19.5,
        gyro_zupt_thresh=0.09,
        acc_zupt_thresh=0.23,
    ):
        self.stamp = init_stamp

        ### state
        self.p = init_pose[:3, 3]
        self.v = init_vel
        self.R = init_pose[:3, :3]
        self.ba = init_ba
        self.bg = init_bg
        ###

        self.g = g

        self.acc_sat = acc_sat

        self.gyro_zupt_thresh = gyro_zupt_thresh
        self.acc_zupt_thresh = acc_zupt_thresh
        self.stationary = True
        self.zupt_stamp = init_stamp
        self.zupt_window = 0.1  # seconds

        self.update_t_sigma = update_t_sigma
        self.update_r_sigma = update_r_sigma

        self.acc_std = acc_std
        self.gyro_std = gyro_std

        self.bacc_std = bacc_std
        self.bgyro_std = bgyro_std

        self.P = np.eye(15, 15) * init_cov

        self.Q = np.eye(6, 6)
        self.Q[:3, :3] *= self.acc_std
        self.Q[3:, 3:] *= self.gyro_std

        sym_inputs_fwd, sym_outputs = EKF.generate_fwd_model()
        p_expr, v_expr, R_expr = sym_outputs

        self.p_fwd_fcn = sp.lambdify(sym_inputs_fwd, p_expr, modules="numpy", cse=True)
        self.v_fwd_fcn = sp.lambdify(sym_inputs_fwd, v_expr, modules="numpy", cse=True)
        self.R_fwd_fcn = sp.lambdify(sym_inputs_fwd, R_expr, modules="numpy", cse=True)

        sym_inputs_up, sym_outputs_up = EKF.generate_update_fcn()
        p_res_expr, R_res_expr = sym_outputs_up

        self.p_residual_fcn = sp.lambdify(
            sym_inputs_up, p_res_expr, modules="numpy", cse=True
        )
        self.R_residual_fcn = sp.lambdify(
            sym_inputs_up, R_res_expr, modules="numpy", cse=True
        )

        if not os.path.exists("jacobians.pkl"):
            F_sym, G_sym = EKF.generate_fwd_jacobians(sym_inputs_fwd, sym_outputs)
            H_sym, J_reset_sym = EKF.generate_update_jacobians(
                sym_inputs_up, sym_outputs_up
            )
            pickle.dump(
                (F_sym, G_sym, H_sym, J_reset_sym),
                open("jacobians.pkl", "wb"),
            )
        else:
            F_sym, G_sym, H_sym, J_reset_sym = pickle.load(open("jacobians.pkl", "rb"))
        self.F_fcn = sp.lambdify(sym_inputs_fwd, F_sym, modules="numpy", cse=True)
        self.G_fcn = sp.lambdify(sym_inputs_fwd, G_sym, modules="numpy", cse=True)
        self.H_fcn = sp.lambdify(sym_inputs_up, H_sym, modules="numpy", cse=True)
        self.J_reset_fcn = sp.lambdify(
            sym_inputs_up, J_reset_sym, modules="numpy", cse=True
        )

    def get_pose(self):
        pose = np.eye(4, 4)
        pose[:3, :3] = self.R
        pose[:3, 3] = self.p
        return pose

    @staticmethod
    def spSkew(v):
        return sp.Matrix(
            [
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0],
            ]
        )

    @staticmethod
    def spUnskew(M):
        return sp.Matrix([M[2, 1], M[0, 2], M[1, 0]])

    @staticmethod
    def spExpSO3(w: sp.Matrix, jac=True):
        theta2 = w.dot(w)
        theta = sp.sqrt(theta2)
        W = EKF.spSkew(w)
        if not jac:
            A = sp.Piecewise((sp.sin(theta) / theta, theta != 0), (1, True))  # type: ignore
            B = sp.Piecewise(
                ((1 - sp.cos(theta)) / theta2, theta != 0), (0.5, True)  # type: ignore
            )

            return sp.eye(3) + A * W + B * W * W
        else:
            return sp.eye(3) + W

    @staticmethod
    def spLogSO3(R: sp.Matrix, jac=True):
        tr = R.trace()
        cos_theta = (tr - 1) / 2
        theta = sp.acos(cos_theta)

        S = (R - R.T) / 2

        if not jac:
            factor = sp.Piecewise(
                (theta / (sp.sin(theta)), theta != 0), (1, True)  # type: ignore
            )
        else:
            factor = 1  # type: ignore

        return factor * EKF.spUnskew(S)

    @staticmethod
    def fix_rotation(R):
        # SVD is the most robust way to project back to SO(3)
        U, _, Vt = np.linalg.svd(R)
        return U @ Vt

    def propagate(self, imu_stamp, lin_acc, ang_vel):
        if self.stamp > imu_stamp:
            print("stamp is older than filter stamp")
            return
        dt = imu_stamp - self.stamp

        # check for zupt
        a_norm = np.linalg.norm(self.R @ (lin_acc - self.ba) + self.g)
        w_norm = np.linalg.norm(ang_vel - self.bg)

        if w_norm < self.gyro_zupt_thresh and a_norm < self.acc_zupt_thresh:
            if not self.stationary:
                self.stationary = True
                self.zupt_stamp = imu_stamp
            elif imu_stamp - self.zupt_stamp > self.zupt_window:
                print("zupt at ", imu_stamp)
                self.zero_velocity_update()
                self.zupt_stamp = imu_stamp
        else:
            self.stationary = False

        saturated = False
        if np.any(lin_acc > self.acc_sat):
            lin_acc = np.zeros((3, 1))
            saturated = True

        inputs = (
            self.p.reshape(3, 1),
            self.v.reshape(3, 1),
            self.R,
            self.ba.reshape(3, 1),
            self.bg.reshape(3, 1),
            lin_acc.reshape(3, 1),
            ang_vel.reshape(3, 1),
            dt,
            self.g.reshape(3, 1),
        )
        new_p = self.p_fwd_fcn(*inputs).flatten()
        new_v = self.v_fwd_fcn(*inputs).flatten()
        new_R = self.R_fwd_fcn(*inputs).flatten()

        self.p = new_p
        self.v = new_v
        self.R = new_R.reshape(3, 3)

        self.R = EKF.fix_rotation(self.R)
        self.stamp = imu_stamp

        F = self.F_fcn(*inputs)
        G = self.G_fcn(*inputs)

        bias_cov = np.zeros((15, 15))
        bias_cov[9:12, 9:12] = np.eye(3, 3) * self.bacc_std * dt
        bias_cov[12:, 12:] = np.eye(3, 3) * self.bgyro_std * dt

        self.P = F @ self.P @ F.T + (G @ self.Q @ G.T / dt) + bias_cov

    @staticmethod
    def skew(w):
        """
        Returns the skew-symmetric matrix of a vector w.
        """
        return np.array(
            [
                [0, -w[2], w[1]],
                [w[2], 0, -w[0]],
                [-w[1], w[0], 0],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def expSO3(w: np.ndarray):
        w = w.flatten()
        theta2 = w.dot(w)
        theta = np.sqrt(theta2)
        W = EKF.skew(w)
        if theta2 < 1e-8:
            return np.eye(3, 3) + W
        else:
            K = W / theta
            one_minus_cos = 2 * np.sin(theta / 2) * np.sin(theta / 2)
            return np.eye(3, 3) + np.sin(theta) * K + one_minus_cos * K * K

    def update(
        self,
        pose_meas,
    ):
        p_up = pose_meas[:3, 3].reshape(3, 1)
        R_up = pose_meas[:3, :3]

        inputs = (self.p.reshape(3, 1), self.R, p_up, R_up)

        p_res = self.p_residual_fcn(*inputs).reshape((-1, 1))
        R_res = self.R_residual_fcn(*inputs).reshape((-1, 1))
        r = np.vstack((p_res, R_res))

        H = self.H_fcn(*inputs)

        R_noise = np.eye(6)
        R_noise[:3, :3] *= self.update_t_sigma**2
        R_noise[3:, 3:] *= self.update_r_sigma**2

        S = H @ self.P @ H.T + R_noise

        K = self.P @ H.T @ np.linalg.inv(S)

        dx = (K @ r).flatten()

        I = np.eye(15)
        self.P = (I - K @ H) @ self.P
        self.P = (self.P + self.P.T) / 2.0

        dp = dx[0:3]
        dv = dx[3:6]
        dR = dx[6:9]
        dba = dx[9:12]
        dbg = dx[12:15]

        self.p = self.p + dp
        self.v = self.v + dv
        self.R = self.R @ EKF.expSO3(dR)
        self.ba = self.ba + dba
        self.bg = self.bg + dbg

        self.R = EKF.fix_rotation(self.R)

        J_reset = self.J_reset_fcn(*inputs)
        self.P = J_reset @ self.P @ J_reset.T

    def zero_velocity_update(self, var=1e-3):
        r = -self.v

        H = -np.zeros((3, 15))
        H[:, 3:6] = np.eye(3, 3)

        R_zupt = np.eye(3) * var

        S = H @ self.P @ H.T + R_zupt

        K = self.P @ H.T @ np.linalg.inv(S)

        dx = (K @ r).flatten()

        I = np.eye(15)
        self.P = (I - K @ H) @ self.P
        self.P = (self.P + self.P.T) / 2.0

        dp = dx[0:3]
        dv = dx[3:6]
        dR = dx[6:9]
        dba = dx[9:12]
        dbg = dx[12:15]

        self.p = self.p + dp
        self.v = self.v + dv
        self.R = self.R @ EKF.expSO3(dR)
        self.ba = self.ba + dba
        self.bg = self.bg + dbg

        self.R = EKF.fix_rotation(self.R)

    @staticmethod
    def generate_fwd_model():
        p_sym = sp.MatrixSymbol("p", 3, 1)
        v_sym = sp.MatrixSymbol("v", 3, 1)
        R_sym = sp.MatrixSymbol("R", 3, 3)
        ba_sym = sp.MatrixSymbol("ba", 3, 1)
        bg_sym = sp.MatrixSymbol("bg", 3, 1)

        a_sym = sp.MatrixSymbol("a", 3, 1)
        w_sym = sp.MatrixSymbol("w", 3, 1)

        g_sym = sp.MatrixSymbol("g", 3, 1)
        p = sp.Matrix(p_sym)
        v = sp.Matrix(v_sym)
        R = sp.Matrix(R_sym)
        ba = sp.Matrix(ba_sym)
        bg = sp.Matrix(bg_sym)
        a = sp.Matrix(a_sym)
        w = sp.Matrix(w_sym)
        g = sp.Matrix(g_sym)
        dt = sp.symbols("dt")

        a_world = R * (a - ba) + g
        p_new = p + v * dt + 0.5 * a_world * dt * dt
        v_new = v + a_world * dt

        R_new = R * EKF.spExpSO3((w - bg) * dt)

        inputs = (p_sym, v_sym, R_sym, ba_sym, bg_sym, a_sym, w_sym, dt, g_sym)
        outputs = (p_new, v_new, R_new)

        return inputs, outputs

    @staticmethod
    def generate_fwd_jacobians(inputs, outputs):
        (p, v, R, ba, bg, a, w, dt, g) = inputs
        (p_new, v_new, R_new) = outputs

        dR_sym = sp.MatrixSymbol("dR", 3, 1)
        dp_sym = sp.MatrixSymbol("dp", 3, 1)
        dv_sym = sp.MatrixSymbol("dv", 3, 1)
        dba_sym = sp.MatrixSymbol("dba", 3, 1)
        dbg_sym = sp.MatrixSymbol("dbg", 3, 1)

        da_sym = sp.MatrixSymbol("da", 3, 1)
        dw_sym = sp.MatrixSymbol("dw", 3, 1)

        dp = sp.Matrix(dp_sym)
        dv = sp.Matrix(dv_sym)
        dR = sp.Matrix(dR_sym)
        dba = sp.Matrix(dba_sym)
        dbg = sp.Matrix(dbg_sym)
        da = sp.Matrix(da_sym)
        dw = sp.Matrix(dw_sym)

        subs_dict = {
            p: p + dp,
            v: v + dv,
            R: R * EKF.spExpSO3(dR),
            ba: ba + dba,
            bg: bg + dbg,
            a: a + da,
            w: w + dw,
        }

        p_new_pert = p_new.subs(subs_dict)
        v_new_pert = v_new.subs(subs_dict)
        R_new_pert = R_new.subs(subs_dict)

        # perturbation forward_model
        p_res = sp.simplify(p_new_pert - p_new)
        v_res = sp.simplify(v_new_pert - v_new)
        R_res = sp.simplify(EKF.spLogSO3(R_new.transpose() * R_new_pert))
        res = sp.Matrix.vstack(p_res, v_res, R_res, dba, dbg)
        dx = sp.Matrix.vstack(dp, dv, dR, dba, dbg)
        du = sp.Matrix.vstack(da, dw)

        print("computing jacobians")
        F = sp.simplify(res.jacobian(dx))
        G = sp.simplify(res.jacobian(du))

        zero_subs = {
            dp_sym: sp.zeros(3, 1),
            dv_sym: sp.zeros(3, 1),
            dR_sym: sp.zeros(3, 1),
            dba_sym: sp.zeros(3, 1),
            dbg_sym: sp.zeros(3, 1),
            da_sym: sp.zeros(3, 1),
            dw_sym: sp.zeros(3, 1),
        }
        print("substituting jacobians")
        F = (F).xreplace(zero_subs)
        G = (G).xreplace(zero_subs)
        print("done")
        return F, G

    @staticmethod
    def generate_update_fcn():
        p_sym = sp.MatrixSymbol("p", 3, 1)
        R_sym = sp.MatrixSymbol("R", 3, 3)
        p = sp.Matrix(p_sym)
        R = sp.Matrix(R_sym)

        R_up_sym = sp.MatrixSymbol("R_up", 3, 3)
        p_up_sym = sp.MatrixSymbol("p_up", 3, 1)
        R_up = sp.Matrix(R_up_sym)
        p_up_sym = sp.Matrix(p_up_sym)

        inputs = (p_sym, R_sym, p_up_sym, R_up_sym)

        return inputs, (p_up_sym - p, EKF.spLogSO3(R.transpose() * R_up))

    @staticmethod
    def generate_update_jacobians(inputs, outputs):
        (p_sym, R_sym, p_up_sym, R_up_sym) = inputs

        p_res, R_res = outputs

        residual = sp.Matrix.vstack(p_res, R_res)

        dp_sym = sp.MatrixSymbol("dp", 3, 1)
        dR_sym = sp.MatrixSymbol("dR", 3, 1)

        dp = sp.Matrix(dp_sym)
        dR = sp.Matrix(dR_sym)

        p = sp.Matrix(p_sym)
        R = sp.Matrix(R_sym)

        subs_dict = {
            p_sym: p + dp,
            R_sym: R * EKF.spExpSO3(dR),
        }

        residual_pert = residual.subs(subs_dict)

        # Residual difference
        res = sp.simplify(residual - residual_pert)

        # Full error state (15D)
        dv_sym = sp.MatrixSymbol("dv", 3, 1)
        dba_sym = sp.MatrixSymbol("dba", 3, 1)
        dbg_sym = sp.MatrixSymbol("dbg", 3, 1)

        dx = sp.Matrix.vstack(
            dp,
            sp.Matrix(dv_sym),
            dR,
            sp.Matrix(dba_sym),
            sp.Matrix(dbg_sym),
        )

        # Measurement Jacobian
        H = sp.simplify(res.jacobian(dx))

        # Zero perturbation substitution
        zero_subs = {
            dp_sym: sp.zeros(3, 1),
            dR_sym: sp.zeros(3, 1),
            dv_sym: sp.zeros(3, 1),
            dba_sym: sp.zeros(3, 1),
            dbg_sym: sp.zeros(3, 1),
        }

        H = H.xreplace(zero_subs)

        J_reset = sp.eye(15)

        return H, J_reset


def run_ekf(
    imu_stamps,
    ang_vels,
    lin_accs,
    pose_stamps,
    poses,
    g=np.array([0.0, 0.0, -9.81]),
    init_ba=np.zeros((3,)),
    init_bg=np.zeros((3,)),
):
    """
    imu_stamps Nx1 np array
    ang_vels Nx3 np array
    lin_accs Nx3 np.array
    pose_stamps Mx1 np.array
    poses Mx4x4 np.array
    """

    output_poses = []
    output_stamps = []

    # stamp = pose_stamps[0]
    stamp = pose_stamps[0]

    pose0 = poses[0, :, :]
    output_poses.append(pose0)
    output_stamps.append(stamp)

    ekf = EKF(
        init_pose=pose0,
        init_stamp=stamp,
        g=g,
        init_ba=init_ba,
        init_bg=init_bg,
    )

    imu_idx = 0
    pose_idx = 1

    while imu_idx < len(imu_stamps) - 1 and pose_idx < len(pose_stamps) - 1:

        current_imu_time = imu_stamps[imu_idx]
        next_pose_time = pose_stamps[pose_idx]

        if current_imu_time < next_pose_time:
            # print("imu")
            imu_idx += 1
            ekf.propagate(
                current_imu_time,
                lin_acc=lin_accs[imu_idx, :],
                ang_vel=ang_vels[imu_idx, :],
            )

        else:
            # print("pose")
            if pose_idx % 5 == 0:
                ekf.update(poses[pose_idx])

            stamp = next_pose_time
            pose_idx += 1

        output_poses.append(ekf.get_pose())
        output_stamps.append(ekf.stamp)
    print("biases at the end")
    print(ekf.ba)
    print(ekf.bg)

    return np.array(output_poses), np.array(output_stamps)


def quat_to_rot_matrix(qx, qy, qz, qw):
    return R_scipy.from_quat([qx, qy, qz, qw]).as_matrix()


def load_gt_tum(filepath: str, imu_in_lidar: np.ndarray):

    data = []

    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.replace("\x00", "")
            if line.strip().startswith("#") or not line.strip():
                continue
            parts = [float(x) for x in line.strip().split()]
            if len(parts) >= 8:
                data.append(parts)

    data = np.array(data)
    data = data[data[:, 0].argsort()]

    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quats = data[:, 4:8]  # qx, qy, qz, qw

    # convert to poses
    poses_lidar = np.zeros((data.shape[0], 4, 4))
    poses_imu = np.zeros((data.shape[0], 4, 4))

    for i in range(data.shape[0]):
        pi = positions[i, :]
        qi = quats[i, :]
        poses_lidar[i, :3, :3] = quat_to_rot_matrix(qi[0], qi[1], qi[2], qi[3])
        poses_lidar[i, :3, 3] = pi

        # poses_imu[i, :4, :4] = (
        #     np.linalg.inv(imu_in_lidar) @ poses_lidar[i, :4, :4] @ imu_in_lidar
        # )

        poses_imu[i, :4, :4] = poses_lidar[i, :4, :4] @ imu_in_lidar

    return timestamps, poses_lidar, poses_imu


def load_imu(file_path):
    stamps = []
    ang_vels = []
    lin_accs = []
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            first_line = f.readline()
            if not first_line:
                return []

            reader = csv.reader((line.replace("\x00", "") for line in f))
            for row in reader:
                rowf = [float(e) for e in row]
                t, wx, wy, wz, ax, ay, az = rowf
                stamps.append(t)
                ang_vels.append(np.array([wx, wy, wz]))
                lin_accs.append(np.array([ax, ay, az]))
    except Exception as e:
        print(f"Error reading IMU file: {e}")
        return []

    return np.array(stamps), np.array(ang_vels), np.array(lin_accs)


def plot_trajectories(
    poses1: np.ndarray,
    poses2: np.ndarray,
    label1="Set 1",
    label2="gt",
    plot_frames=False,
    frame_scale=1.0,
    stride=1,
):
    """
    Plots the translation component of two sets of 4x4 isometries.
    Optionally plots RGB reference frames (X=Red, Y=Green, Z=Blue).

    Args:
        poses1: (N, 4, 4) numpy array of poses
        poses2: (M, 4, 4) numpy array of poses
        label1: String label for the first set
        label2: String label for the second set
        plot_frames: Boolean, if True, plots the full pose (axes)
        frame_scale: Length of the coordinate frame axes
        stride: Int, step size for plotting frames (e.g. 10 plots every 10th frame)
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # --- Helper to plot coordinate frames ---
    def plot_pose_frames(poses, length):
        # Filter by stride to avoid overcrowding
        subset = poses[::stride]

        # Origins (x, y, z)
        o_x = subset[:, 0, 3]
        o_y = subset[:, 1, 3]
        o_z = subset[:, 2, 3]

        # Rotation columns: 0=x (Red), 1=y (Green), 2=z (Blue)
        # Plot X-axis (Red)
        ax.quiver(
            o_x,
            o_y,
            o_z,
            subset[:, 0, 0],
            subset[:, 1, 0],
            subset[:, 2, 0],
            color="r",
            length=length,
            normalize=True,
            alpha=0.6,
        )
        # Plot Y-axis (Green)
        ax.quiver(
            o_x,
            o_y,
            o_z,
            subset[:, 0, 1],
            subset[:, 1, 1],
            subset[:, 2, 1],
            color="g",
            length=length,
            normalize=True,
            alpha=0.6,
        )
        # Plot Z-axis (Blue)
        ax.quiver(
            o_x,
            o_y,
            o_z,
            subset[:, 0, 2],
            subset[:, 1, 2],
            subset[:, 2, 2],
            color="b",
            length=length,
            normalize=True,
            alpha=0.6,
        )

    # ----------------------------------------

    # Plot Trajectory Lines
    ax.plot(
        poses1[:, 0, 3],
        poses1[:, 1, 3],
        poses1[:, 2, 3],
        c="blue",
        label=label1,
        linewidth=1,
        alpha=0.8,
    )

    ax.plot(
        poses2[:, 0, 3],
        poses2[:, 1, 3],
        poses2[:, 2, 3],
        c="red",
        label=label2,
        linewidth=1,
        alpha=0.8,
    )

    # Plot Frames if requested
    if plot_frames:
        plot_pose_frames(poses1, frame_scale)
        # plot_pose_frames(poses2, frame_scale)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()

    # --- Auto-scale axes to equal aspect ratio ---
    all_x = poses1[:, 0, 3]
    all_y = poses1[:, 1, 3]
    all_z = poses1[:, 2, 3]

    max_range = (
        np.array(
            [
                all_x.max() - all_x.min(),
                all_y.max() - all_y.min(),
                all_z.max() - all_z.min(),
            ]
        ).max()
        / 2.0
    )

    mid_x = (all_x.max() + all_x.min()) * 0.5
    mid_y = (all_y.max() + all_y.min()) * 0.5
    mid_z = (all_z.max() + all_z.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    # ---------------------------------------------


def plot_component_comparison(
    poses1: np.ndarray,
    stamps1: np.ndarray,
    poses2: np.ndarray,
    stamps2: np.ndarray,
    label1="Set 1",
    label2="gt",
):
    """
    Plots the individual components (x, y, z) and Euler angles (roll, pitch, yaw)
    of two sets of 4x4 isometries against their specific timestamps.
    """

    # Ensure timestamps are 1D arrays for plotting
    t1 = stamps1.flatten()
    t2 = stamps2.flatten()

    # --- 1. Extract Position Components ---
    x1, y1, z1 = poses1[:, 0, 3], poses1[:, 1, 3], poses1[:, 2, 3]
    x2, y2, z2 = poses2[:, 0, 3], poses2[:, 1, 3], poses2[:, 2, 3]

    # --- 2. Extract Rotation (Euler Angles) ---
    rot1 = R_scipy.from_matrix(poses1[:, :3, :3])
    rot2 = R_scipy.from_matrix(poses2[:, :3, :3])

    # Convert to Euler 'xyz' in degrees
    euler1 = rot1.as_euler("xyz", degrees=True)
    euler2 = rot2.as_euler("xyz", degrees=True)

    r1, p1, yw1 = euler1[:, 0], euler1[:, 1], euler1[:, 2]
    r2, p2, yw2 = euler2[:, 0], euler2[:, 1], euler2[:, 2]

    # --- 3. Plotting ---
    fig, axs = plt.subplots(2, 3, figsize=(18, 8), sharex=True)
    fig.suptitle(f"Trajectory Component Comparison: {label1} vs {label2}")

    # Helper function to plot data against time
    def plot_ax(ax, time1, data1, time2, data2, title, ylabel):
        ax.plot(time1, data1, label=label1, color="blue", alpha=0.8, linewidth=1.5)
        ax.plot(
            time2,
            data2,
            label=label2,
            color="red",
            alpha=0.8,
            marker="*",
            linestyle="None",
            markersize=3,
        )
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, which="both", linestyle="--", alpha=0.5)
        ax.legend()

    # -- Row 1: Positions --
    plot_ax(axs[0, 0], t1, x1, t2, x2, "Position X", "m")
    plot_ax(axs[0, 1], t1, y1, t2, y2, "Position Y", "m")
    plot_ax(axs[0, 2], t1, z1, t2, z2, "Position Z", "m")

    # -- Row 2: Orientations --
    plot_ax(axs[1, 0], t1, r1, t2, r2, "Roll (Euler X)", "degrees")
    plot_ax(axs[1, 1], t1, p1, t2, p2, "Pitch (Euler Y)", "degrees")
    plot_ax(axs[1, 2], t1, yw1, t2, yw2, "Yaw (Euler Z)", "degrees")

    # Set common X label
    for ax in axs[1, :]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()


def plot_imu_analysis(
    imu_stamps: np.ndarray,
    lin_accs: np.ndarray,
    ang_vels: np.ndarray,
    thresh_acc=None,
    thresh_gyro=None,
):
    """
    Plots IMU data: 3 components for Linear Acc and Angular Vel, plus their norms.

    Args:
        imu_stamps: (N, 1) or (N,) array of timestamps
        lin_accs: (N, 3) array of linear accelerations
        ang_vels: (N, 3) array of angular velocities
        thresh_acc: (Optional) Float, draws a horizontal line on Acc Norm plot
        thresh_gyro: (Optional) Float, draws a horizontal line on Gyro Norm plot
    """

    # Ensure 1D time array
    t = imu_stamps.flatten()

    # Safety check for shapes
    if len(t) != len(lin_accs) or len(t) != len(ang_vels):
        print(f"Warning: resizing to minimum length.")
        min_len = min(len(t), len(lin_accs), len(ang_vels))
        t = t[:min_len]
        lin_accs = lin_accs[:min_len]
        ang_vels = ang_vels[:min_len]

    # Calculate Norms
    acc_norm = np.linalg.norm(lin_accs, axis=1)
    gyro_norm = np.linalg.norm(ang_vels, axis=1)

    # Create Plot (4 Rows x 2 Columns)
    # Rows 0-2: X, Y, Z components
    # Row 3: Norms
    fig, axs = plt.subplots(4, 2, figsize=(14, 12), sharex=True)
    fig.suptitle("IMU Raw Data Analysis", fontsize=16)

    # --- Helper for standard plotting ---
    def plot_signal(ax, time, data, color, title, ylabel):
        ax.plot(time, data, color=color, linewidth=1, alpha=0.8)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.grid(True, which="both", linestyle="--", alpha=0.5)

    # --- Linear Acceleration (Left Column) ---
    plot_signal(axs[0, 0], t, lin_accs[:, 0], "tab:red", "Accel X", "m/s²")
    plot_signal(axs[1, 0], t, lin_accs[:, 1], "tab:red", "Accel Y", "m/s²")
    plot_signal(axs[2, 0], t, lin_accs[:, 2], "tab:red", "Accel Z", "m/s²")

    # Accel Norm
    plot_signal(axs[3, 0], t, acc_norm, "k", "Accel Norm", "m/s²")
    if thresh_acc is not None:
        axs[3, 0].axhline(
            y=thresh_acc,
            color="r",
            linestyle="--",
            linewidth=1.5,
            label=f"Thresh: {thresh_acc}",
        )
        axs[3, 0].legend(loc="upper right")

    # --- Angular Velocity (Right Column) ---
    plot_signal(axs[0, 1], t, ang_vels[:, 0], "tab:blue", "Gyro X", "rad/s")
    plot_signal(axs[1, 1], t, ang_vels[:, 1], "tab:blue", "Gyro Y", "rad/s")
    plot_signal(axs[2, 1], t, ang_vels[:, 2], "tab:blue", "Gyro Z", "rad/s")

    # Gyro Norm
    plot_signal(axs[3, 1], t, gyro_norm, "k", "Gyro Norm", "rad/s")
    if thresh_gyro is not None:
        axs[3, 1].axhline(
            y=thresh_gyro,
            color="r",
            linestyle="--",
            linewidth=1.5,
            label=f"Thresh: {thresh_gyro}",
        )
        axs[3, 1].legend(loc="upper right")

    # X-axis label only on bottom row
    axs[3, 0].set_xlabel("Time (s)")
    axs[3, 1].set_xlabel("Time (s)")

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)  # Make space for suptitle
    plt.show()


def estimate_biases(imu_stamps, lin_accs, ang_vels: np.ndarray, gyro_std_thresh=0.005):
    """
    Try to estimate biases by looking at the end of the sequence where I stay still for a while
    """

    ang_norms = np.linalg.norm(ang_vels, axis=1)

    # compute variance of angular velocities with a rolling filter
    window_size = 11
    k = np.ones((window_size,)) / float(window_size)

    exp_x_sq = np.convolve(ang_norms**2, k, mode="valid")
    exp_sq_x = np.convolve(ang_norms, k, mode="valid") ** 2

    std = np.sqrt(exp_x_sq - exp_sq_x)

    still_frames = std < gyro_std_thresh

    # pad to fit imu_stamp

    offset = int((len(imu_stamps) - len(still_frames)) / 2)
    still_frames = np.pad(still_frames, offset, mode="constant", constant_values=True)

    gyro_moving_average: List[np.ndarray] = []

    for i in range(len(still_frames) - offset):
        idx = len(still_frames) - i - 1

        if still_frames[idx]:
            gyro_moving_average.append(ang_vels[idx])
        else:
            break

    gyro_moving_average_np = np.array(gyro_moving_average)

    return None, np.mean(gyro_moving_average_np, axis=0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("imu_file", help="Path to imu txt file")
    parser.add_argument("gt_file", help="Path to estimate.txt (TUM format)")
    parser.add_argument("tf_file", help="Path to the imu in lidar instrinsics")
    parser.add_argument(
        "--correct_time", type=float, default=0.1, help="Seconds between GT corrects"
    )
    args = parser.parse_args()

    bg_val = np.array([0.0, 0.0, 0.0])
    g_val = np.array([0.0, 0.0, -9.8])

    gt_stamps, gt_lidar, gt_imu = load_gt_tum(args.gt_file, np.loadtxt(args.tf_file))

    imu_stamps, ang_vels, lin_accs = load_imu(args.imu_file)

    estimate_biases(imu_stamps, lin_accs, ang_vels)
    # # plot_trajectories(gt_lidar, gt_imu, "lidar", "imu", plot_frames=False)

    guess_ba, guess_bg = estimate_biases(imu_stamps, lin_accs, ang_vels)

    print("Estimated biases")
    print(guess_ba)
    print(guess_bg)

    output_poses, output_stamps = run_ekf(
        imu_stamps,
        ang_vels,
        lin_accs,
        gt_stamps,
        gt_imu,
        init_ba=np.array([0.1226277, 0.17389153, -0.13508909]),
        init_bg=guess_bg,
    )

    print(output_poses)

    plot_trajectories(output_poses, gt_imu)
    plot_component_comparison(output_poses, output_stamps, gt_imu, gt_stamps)
    plot_imu_analysis(imu_stamps, lin_accs, ang_vels)

    plt.show()
