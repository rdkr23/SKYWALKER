import numpy as np
from scipy.spatial.transform import Rotation


def rpy_global_to_rpy_euler(rpy):
    r_zyx = Rotation.from_euler('xyz', rpy)

    r_xyz = r_zyx.as_euler('xyz', degrees=False)

    return [r_xyz[2], r_xyz[1], r_xyz[0]]

def rpy_euler_to_rpy_global(rpy):
    r_xyz = Rotation.from_euler('xyz', rpy)

    r_zyx = r_xyz.as_euler('xyz', degrees=False)

    return [r_zyx[0], r_zyx[1], r_zyx[2]]

def rpy_to_tm(pose):
    r = Rotation.from_euler('xyz', [pose[3], pose[4], pose[5]])
    r = r.as_matrix()

    tm = np.vstack((np.hstack((r, [[pose[0]], [pose[1]], [pose[2]]])), [0, 0, 0, 1]))

    return tm

def tc(quat):
    return Rotation.from_quat(quat).as_euler('xyz')


def tm_to_rpy(tm):
    x = tm[0, 3]
    y = tm[1, 3]
    z = tm[2, 3]

    rot = tm[:3, :3]

    r = Rotation.from_matrix(rot)
    resp = [x, y, z] + r.as_euler('xyz').tolist()

    return resp

def rpy_to_quat(rpy):
    return Rotation.from_euler('xyz', rpy).as_quat()

def euler_rpy_to_quat(rpy):
    return Rotation.from_euler('xyz', rpy).as_quat()

def quat_to_rpy(quat):
    return Rotation.from_quat(quat).as_euler('xyz')

def quad_to_tm(pose):
    r = Rotation.from_quat([pose[3], pose[4], pose[5], pose[6]])
    r = r.as_matrix()

    tm = np.vstack((np.hstack((r, [[pose[0]], [pose[1]], [pose[2]]])), [0, 0, 0, 1]))

    return tm


def tm_to_quad(tm):
    x = tm[0, 3]
    y = tm[1, 3]
    z = tm[2, 3]

    rot = tm[:3, :3]

    r = Rotation.from_matrix(rot)
    resp = [x, y, z] + r.as_quat().tolist()

    return resp
