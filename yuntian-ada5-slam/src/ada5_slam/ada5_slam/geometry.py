"""SE(2) helpers. Poses are arrays (x, y, theta) in metres and radians."""
import numpy as np


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rot2(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


def compose(a, b):
    """a then b: the pose b expressed in a's frame, mapped to a's parent frame."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])])


def inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])


def between(a, b):
    """Pose of b in the frame of a."""
    return compose(inverse(a), b)


def transform(pose, pts):
    """Apply a pose to N x 2 points (element-wise: small BLAS products are very slow on the VM)."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    x, y = pts[:, 0], pts[:, 1]
    return np.column_stack([c * x - s * y + pose[0], s * x + c * y + pose[1]])


def quat_matrix(q):
    """Rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))
