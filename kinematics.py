"""URDF-based FK and damped-least-squares full-pose IK using NumPy."""
import xml.etree.ElementTree as ET
import numpy as np


def rpy_rotation(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def homogeneous(rot=np.eye(3), xyz=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


def axis_rotation(axis, theta):
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
                     [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
                     [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)]])


def vec(text, default):
    return np.array([float(v) for v in (text or default).split()], dtype=float)


class PandaURDF:
    def __init__(self, xml_text):
        root = ET.fromstring(xml_text)
        all_joints = {}
        for j in root.findall('joint'):
            child = j.find('child').attrib['link']
            origin = j.find('origin')
            xyz = vec(origin.get('xyz') if origin is not None else None, '0 0 0')
            rpy = vec(origin.get('rpy') if origin is not None else None, '0 0 0')
            axis_node = j.find('axis')
            axis = vec(axis_node.get('xyz') if axis_node is not None else None, '1 0 0')
            lim = j.find('limit')
            lower = float(lim.get('lower', '-3.1')) if lim is not None else -3.1
            upper = float(lim.get('upper', '3.1')) if lim is not None else 3.1
            all_joints[child] = dict(parent=j.find('parent').get('link'),
                                     child=child, name=j.get('name'), type=j.get('type'),
                                     origin=homogeneous(rpy_rotation(rpy), xyz), axis=axis,
                                     lower=lower, upper=upper)
        # The Panda model's named terminal links vary across versions.
        for ee in ('panda_hand', 'panda_link8', 'panda_link7'):
            if ee in all_joints:
                break
        else:
            raise ValueError('Cannot find Panda terminal frame in URDF')
        path, link = [], ee
        while link in all_joints:
            joint = all_joints[link]
            path.insert(0, joint)
            link = joint['parent']
        self.path = path
        self.movable = [j for j in path if j['type'] in ('revolute', 'continuous')]
        if len(self.movable) != 7:
            raise ValueError(f'Expected 7 movable Panda arm joints, found {len(self.movable)}')
        self.names = [j['name'] for j in self.movable]
        self.lower = np.array([j['lower'] for j in self.movable])
        self.upper = np.array([j['upper'] for j in self.movable])
        self.ee_frame = ee
        self.base_frame = link

    def fk(self, q):
        t = np.eye(4)
        pos = 0
        for j in self.path:
            t = t @ j['origin']
            if j['type'] in ('revolute', 'continuous'):
                t = t @ homogeneous(axis_rotation(j['axis'], q[pos]))
                pos += 1
            elif j['type'] != 'fixed':
                raise ValueError('Unsupported non-fixed joint')
        return t

    @staticmethod
    def error(target, current):
        # Geometric spatial orientation error in base frame.
        rc, rt = current[:3, :3], target[:3, :3]
        omega = 0.5 * sum(np.cross(rc[:, i], rt[:, i]) for i in range(3))
        return np.r_[target[:3, 3] - current[:3, 3], omega]

    def ik(self, target, q0, max_iterations=280, tolerance=2e-4):
        q = np.clip(np.array(q0, dtype=float), self.lower+0.015, self.upper-0.015)
        h = 1e-5
        for iteration in range(max_iterations):
            e = self.error(target, self.fk(q))
            if np.linalg.norm(e) < tolerance:
                return q, True, iteration, float(np.linalg.norm(e))
            jac = np.empty((6, 7))
            for idx in range(7):
                qh = q.copy()
                qh[idx] += h
                jac[:, idx] = (self.error(target, self.fk(q)) -
                               self.error(target, self.fk(qh))) / h
            damping = 0.06
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping*damping*np.eye(6), e)
            q = np.clip(q + np.clip(dq, -0.10, 0.10), self.lower+0.015, self.upper-0.015)
        return q, False, max_iterations, float(np.linalg.norm(self.error(target, self.fk(q))))
