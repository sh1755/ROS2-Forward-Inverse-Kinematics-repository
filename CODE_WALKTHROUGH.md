# Code walkthrough — Forward & Inverse Kinematics (V4)

This guide explains the **educational ROS 2 FK/IK visualizer**. The older physical-robot C++ code is not part of this repository.

## 1. Architecture

The Python package is expected at `panda_fk_ik_demo/`:

- `panda_fk_ik_demo/kinematics.py`: geometry and kinematics
- `panda_fk_ik_demo/demo.py`: ROS node, task stages, smooth interpolation, finger joint states
- `launch/demo.launch.py`: robot description, visualization and node launch
- `rviz/panda_fk_ik.rviz`: RobotModel and display configuration

The precise class and function signatures should always be checked against the uploaded Python source.

## 2. Forward kinematics (FK)

**Input:** the seven arm joint angles `q` in radians.  
**Output:** a 4×4 homogeneous transformation containing the gripper position and rotation.

For a serial robot:

```math
T_{0}^{EE}(q)=T_0^1(q_1)T_1^2(q_2)\cdots T_7^{EE}(q_7)
```

Example usage:

```python
# q is a vector of 7 joint angles (radians)
pose = arm.fk(q)
position = pose[:3, 3]
rotation = pose[:3, :3]
print("End-effector XYZ (m):", position)
```

**Teaching prompt:** “If the encoder supplies all joint angles, how can we calculate the gripper location?”

## 3. Inverse kinematics (IK)

**Input:** desired 3D pose and a starting joint configuration (seed).  
**Output:** a joint configuration that approximately achieves the requested pose, together with diagnostic information.

```python
q_goal, success, iterations, error = arm.ik(target_pose, q_seed)
if not success:
    print("Numerical IK did not converge")
```

A typical damped-least-squares step is:

```math
\Delta q = J^T(JJ^T + \lambda^2 I)^{-1}e
```

where `e` is a six-dimensional pose error, `J` is the robot's geometric Jacobian, and `λ` provides damping near difficult configurations. Real implementations may also clip updates and respect joint limits.

**Why is IK harder?** There may be no feasible solution, multiple solutions, singularities or redundant degrees of freedom.

## 4. V4 worked demonstration: FK followed by IK

The V4 teaching demo forms a **known reachable target pose** from a predefined configuration, then solves IK to recover a suitable configuration:

```python
q_example = ...                 # Known, feasible 7-joint pose
target_pose = arm.fk(q_example) # Forward kinematics

q_solution, ok, n, residual = arm.ik(
    target_pose, q_seed
)                              # Inverse kinematics
```

The resulting `q_solution` may differ from `q_example` because a redundant manipulator can have different valid joint configurations for the same end-effector pose.

This is a **pedagogical FK → IK cycle**, rather than a camera-driven autonomous target selection.

## 5. Interpolation and gripper position

A common smoothstep scalar is:

```math
s(u)=3u^2-2u^3,\quad 0\le u\le 1
```

Joint positions for an animation can be interpolated as:

```python
s = 3.0 * u*u - 2.0 * u*u*u
q = q_start + s * (q_goal - q_start)
```

For an animated two-finger gripper, the joint state vector also contains the two finger positions:

```python
joint_msg.name = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6",
    "panda_joint7", "panda_finger_joint1",
    "panda_finger_joint2"
]
joint_msg.position = list(q) + [finger_position, finger_position]
```

**Important:** Joint interpolation is not guaranteed to produce a straight-line end-effector path.

## 6. ROS 2 visualization data flow

```text
demo.py
   └── /joint_states  [sensor_msgs/JointState]
         └── robot_state_publisher
               ├── /tf
               └── /tf_static
                      └── RViz2 RobotModel / TF display
```

The arm model is loaded from the robot description. RViz renders links based on the current joint and TF information.

Useful checks:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /panda_fk_ik/status
ros2 run tf2_ros tf2_echo panda_link0 panda_hand
```

When RViz is blank, verify that **Fixed Frame** is `panda_link0`, RobotModel is enabled, and its description points to `/robot_description`.

## 7. Task stages, and what they mean

The V4 sequence illustrates: **Home → Approach → Grasp → Close → Lift → Transfer → Place → Release → Retreat → Return.**

These are named animation stages; they do not automatically guarantee a genuine grasp, contact dynamics, obstacle avoidance, or object attachment. The displayed joint states are **simulation visualization only** and must not be sent unchanged to physical hardware.

## 8. Explaining the algorithm in an interview

> “First, I use forward kinematics to compute the tool pose for a known seven-joint configuration. Then inverse kinematics solves for a configuration matching the desired tool pose. A smooth interpolation creates a joint-space animation, and a ROS 2 JointState publisher drives the robot model and two gripper fingers in RViz. This makes the FK and IK mapping visible without actuating the physical robot.”

## 9. What to add next

- A real RViz screen recording under `media/`
- A sample numerical FK/IK table from the running node
- Cartesian target markers
- Unit tests comparing FK and solved IK end-effector poses
- MoveIt 2 planning and collision checks for more realistic robotic manipulation
