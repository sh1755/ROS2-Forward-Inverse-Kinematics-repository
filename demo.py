"""Educational Panda pick-and-place-style FK/IK cycle. RViz only; no robot actuation."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .kinematics import PandaURDF


def quaternion_from_rotation(rot):
    """Return xyzw quaternion from an orthonormal 3x3 rotation matrix."""
    tr = np.trace(rot)
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return [(rot[2, 1] - rot[1, 2]) / s,
                (rot[0, 2] - rot[2, 0]) / s,
                (rot[1, 0] - rot[0, 1]) / s, 0.25 * s]
    k = int(np.argmax(np.diag(rot)))
    a, b, c = k, (k + 1) % 3, (k + 2) % 3
    s = 2.0 * math.sqrt(1.0 + rot[a, a] - rot[b, b] - rot[c, c])
    quat = [0.0, 0.0, 0.0, 0.0]
    quat[a] = 0.25 * s
    quat[b] = (rot[a, b] + rot[b, a]) / s
    quat[c] = (rot[a, c] + rot[c, a]) / s
    quat[3] = (rot[c, b] - rot[b, c]) / s
    return quat


class PandaComplexMotion(Node):
    def __init__(self):
        super().__init__('panda_fk_ik_demo')
        self.declare_parameter('urdf_file', '')
        urdf_file = self.get_parameter('urdf_file').value
        if not urdf_file:
            raise RuntimeError('Provide a Panda URDF path via urdf_file')
        with open(urdf_file, encoding='utf-8') as file:
            self.arm = PandaURDF(file.read())

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/panda_fk_ik/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/panda_fk_ik/status', 10)
        self.arm_names = list(self.arm.names)
        self.gripper_names = ['panda_finger_joint1', 'panda_finger_joint2']
        self.dt = 0.05

        # 7-joint examples produce reachable full 6D tool poses via FK.
        # These named poses illustrate task stages, not verified vertical paths.
        seeds = [
            ('HOME',     [0.00, -0.65, 0.00, -2.10,  0.00, 1.65, 0.78], 0.030, 2.0),
            ('APPROACH', [0.25, -0.60, 0.20, -2.00,  0.05, 1.70, 0.75], 0.030, 3.0),
            ('GRASP',    [0.30, -0.48, 0.27, -1.88, -0.04, 1.84, 0.83], 0.030, 2.5),
            ('CLOSE',    [0.30, -0.48, 0.27, -1.88, -0.04, 1.84, 0.83], 0.000, 1.5),
            ('LIFT',     [0.17, -0.73, 0.10, -2.05,  0.02, 1.65, 0.80], 0.000, 3.0),
            ('TRANSFER',[-0.42, -0.55, -0.12, -1.92, 0.15, 1.79, 0.96], 0.000, 4.0),
            ('PLACE',   [-0.50, -0.39, -0.25, -1.76, 0.13, 1.92, 1.05], 0.000, 2.5),
            ('RELEASE', [-0.50, -0.39, -0.25, -1.76, 0.13, 1.92, 1.05], 0.030, 1.5),
            ('RETREAT', [-0.40, -0.62, -0.10, -1.97, 0.12, 1.74, 0.98], 0.030, 2.5),
            ('RETURN',  [0.00, -0.65, 0.00, -2.10, 0.00, 1.65, 0.78], 0.030, 3.0),
        ]
        self.stages = []
        previous = None
        for label, degrees_like, finger, duration in seeds:
            seed = np.clip(np.array(degrees_like, dtype=float),
                           self.arm.lower + 0.03, self.arm.upper - 0.03)
            # FK: joint seed -> exact full end-effector transform.
            target = self.arm.fk(seed)
            # IK: desired transform -> joint vector; start near previous stage.
            # If numerical IK fails, use the original reachable seed.
            if previous is None:
                q, converged, steps, residual = seed, True, 0, 0.0
            elif np.allclose(seed, self.stages[-1]['seed']):
                q, converged, steps, residual = previous.copy(), True, 0, 0.0
            else:
                q, converged, steps, residual = self.arm.ik(target, previous)
                if not converged:
                    self.get_logger().warning(
                        f'{label}: IK did not converge (residual={residual:.5f}); '
                        'using FK-verified joint seed for this demonstration')
                    q = seed.copy()
            self.stages.append(dict(name=label, seed=seed, q=q,
                                    T=target, finger=finger, seconds=duration,
                                    ik_ok=converged))
            previous = q.copy()
            self.get_logger().info(
                f'{label}: FK xyz={np.round(target[:3, 3], 3).tolist()}, '
                f'IK success={converged}, iterations={steps}, '
                f'joints(deg)={np.round(np.rad2deg(q), 1).tolist()}')

        self.total_time = sum(stage['seconds'] for stage in self.stages)
        self.start_time = self.get_clock().now()
        self.previous_stage = -1
        self.create_timer(self.dt, self.publish_step)
        self.get_logger().info('RViz-only staged FK/IK demonstration started. '
                               'No collision checks, no object physics, NO hardware commands.')

    def publish_step(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        t = elapsed % self.total_time
        stage_index = 0
        for i, stage in enumerate(self.stages):
            if t < stage['seconds']:
                stage_index = i
                break
            t -= stage['seconds']
        stage = self.stages[stage_index]
        prev = self.stages[stage_index - 1] if stage_index else self.stages[-1]
        # The initial stage uses its own position on the very first pass.
        if stage_index == 0 and elapsed < self.stages[0]['seconds']:
            prev = stage
        u = min(max(t / stage['seconds'], 0.0), 1.0)
        smooth = u*u*(3.0 - 2.0*u)  # smoothstep: zero endpoint velocity
        q = prev['q'] + smooth * (stage['q'] - prev['q'])
        finger = prev['finger'] + smooth * (stage['finger'] - prev['finger'])

        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = self.arm_names + self.gripper_names
        js.position = [float(v) for v in q] + [float(finger), float(finger)]
        self.js_pub.publish(js)

        target = stage['T']
        pose = PoseStamped()
        pose.header.frame_id = self.arm.base_frame
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, target[:3, 3])
        quat = quaternion_from_rotation(target[:3, :3])
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = [float(v) for v in quat]
        self.target_pub.publish(pose)
        self.status_pub.publish(String(data=f'{stage_index + 1}/{len(self.stages)}: {stage["name"]}'))
        if stage_index != self.previous_stage:
            self.previous_stage = stage_index
            self.get_logger().info(f'ACTIVE STAGE {stage_index + 1}/{len(self.stages)}: {stage["name"]}')


def main():
    rclpy.init()
    node = None
    try:
        node = PandaComplexMotion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
