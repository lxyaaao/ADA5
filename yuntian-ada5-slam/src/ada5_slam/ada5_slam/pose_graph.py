"""2D pose graph over keyframes, solved with SciPy least squares (Lecture 6 slides 17, 44-45)."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from .geometry import wrap


def optimize(poses, edges, anchor=0, loss='huber', f_scale=1.0, max_nfev=60, tol=1e-6):
    """poses: N x 3 initial keyframe poses. edges: (i, j, z, sqrt_info) with z the measured pose
    of j in i's frame and sqrt_info a 3 x 3 square-root information matrix.

    Residuals are whitened, so the robust loss acts in units of standard deviations.
    """
    poses = np.asarray(poses, float)
    n = len(poses)
    I = np.array([e[0] for e in edges])
    J = np.array([e[1] for e in edges])
    Z = np.array([e[2] for e in edges], float)
    L = np.array([e[3] for e in edges], float)
    ref = poses[anchor].copy()

    def residuals(x):
        P = x.reshape(n, 3)
        pi, pj = P[I], P[J]
        c, s = np.cos(pi[:, 2]), np.sin(pi[:, 2])
        dx, dy = pj[:, 0] - pi[:, 0], pj[:, 1] - pi[:, 1]
        e = np.column_stack([c * dx + s * dy - Z[:, 0], -s * dx + c * dy - Z[:, 1],
                             wrap(pj[:, 2] - pi[:, 2] - Z[:, 2])])
        return np.concatenate([(P[anchor] - ref) * 1e3, np.einsum('eab,eb->ea', L, e).ravel()])

    S = lil_matrix((3 + 3 * len(edges), 3 * n), dtype=int)
    S[0:3, 3 * anchor:3 * anchor + 3] = 1
    for k, (i, j) in enumerate(zip(I, J)):
        S[3 + 3 * k:6 + 3 * k, 3 * i:3 * i + 3] = 1
        S[3 + 3 * k:6 + 3 * k, 3 * j:3 * j + 3] = 1
    res = least_squares(residuals, poses.ravel(), jac_sparsity=S, method='trf', loss=loss,
                        f_scale=f_scale, x_scale='jac', max_nfev=max_nfev, ftol=tol, xtol=tol, gtol=tol)
    out = res.x.reshape(n, 3)
    out[:, 2] = wrap(out[:, 2])
    return out, res


def edge_errors(poses, edges):
    """Translation (m) and heading (deg) disagreement of each edge with the poses."""
    P = np.asarray(poses, float)
    out = []
    for i, j, z, _ in edges:
        c, s = np.cos(P[i, 2]), np.sin(P[i, 2])
        d = P[j, :2] - P[i, :2]
        e = np.array([c * d[0] + s * d[1] - z[0], -s * d[0] + c * d[1] - z[1]])
        out.append((float(np.hypot(*e)), float(np.degrees(abs(wrap(P[j, 2] - P[i, 2] - z[2]))))))
    return np.array(out).reshape(-1, 2)
