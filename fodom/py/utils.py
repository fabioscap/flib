import numpy as np
import sympy as sp


def spSkew(v):
    return sp.Matrix(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )


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


def spUnskew(M):
    return sp.Matrix([M[2, 1], M[0, 2], M[1, 0]])


def spExpSO3(w: sp.Matrix, jac=True):
    theta2 = w.dot(w)
    theta = sp.sqrt(theta2)
    W = spSkew(w)
    if not jac:
        A = sp.Piecewise((sp.sin(theta) / theta, theta != 0), (1, True))  # type: ignore
        B = sp.Piecewise(
            ((1 - sp.cos(theta)) / theta2, theta != 0), (0.5, True)  # type: ignore
        )
        return sp.eye(3) + A * W + B * W * W
    else:
        return sp.eye(3) + W


def expSO3(w: np.ndarray):
    w = w.flatten()
    theta2 = w.dot(w)
    theta = np.sqrt(theta2)
    W = skew(w)
    if theta2 < 1e-8:
        return np.eye(3, 3) + W
    else:
        K = W / theta
        one_minus_cos = 2 * np.sin(theta / 2) * np.sin(theta / 2)
        return np.eye(3, 3) + np.sin(theta) * K + one_minus_cos * K * K


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

    return factor * spUnskew(S)


def fix_rotation(R):
    # SVD is the most robust way to project back to SO(3)
    U, _, Vt = np.linalg.svd(R)
    return U @ Vt
