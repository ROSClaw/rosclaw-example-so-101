from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import Joint, Pose as UrdfPose, URDF


MOVABLE_JOINT_TYPES = {'continuous', 'prismatic', 'revolute'}


@dataclass(frozen=True)
class CartesianIkResult:
    positions: list[float]
    position_error_m: float
    orientation_error_rad: float
    weighted_error: float
    iterations: int
    converged: bool


def frame_id_to_link_name(frame_id: str) -> str:
    stripped = frame_id.strip().strip('/')
    if not stripped:
        raise ValueError('Frame id must not be empty.')
    return stripped.split('/')[-1]


def normalize_pose_stamped(goal: PoseStamped) -> PoseStamped:
    quaternion = goal.pose.orientation
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm <= 1e-9:
        raise ValueError('Goal orientation must be a non-zero quaternion.')

    normalized = PoseStamped()
    normalized.header = goal.header
    normalized.pose.position.x = goal.pose.position.x
    normalized.pose.position.y = goal.pose.position.y
    normalized.pose.position.z = goal.pose.position.z
    normalized.pose.orientation.x = quaternion.x / norm
    normalized.pose.orientation.y = quaternion.y / norm
    normalized.pose.orientation.z = quaternion.z / norm
    normalized.pose.orientation.w = quaternion.w / norm
    return normalized


def pose_to_kdl_frame(pose: Pose) -> kdl.Frame:
    rotation = kdl.Rotation.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    translation = kdl.Vector(pose.position.x, pose.position.y, pose.position.z)
    return kdl.Frame(rotation, translation)


def _urdf_pose_to_kdl_frame(pose: UrdfPose | None) -> kdl.Frame:
    if pose is None:
        return kdl.Frame.Identity()

    xyz = pose.xyz if pose.xyz is not None else [0.0, 0.0, 0.0]
    rpy = pose.rpy if pose.rpy is not None else [0.0, 0.0, 0.0]
    return kdl.Frame(
        kdl.Rotation.RPY(*xyz_to_float_triplet(rpy)),
        kdl.Vector(*xyz_to_float_triplet(xyz)),
    )


def xyz_to_float_triplet(values: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = values
    return float(x), float(y), float(z)


def _jnt_array(values: Iterable[float]) -> kdl.JntArray:
    values_list = [float(value) for value in values]
    joint_array = kdl.JntArray(len(values_list))
    for index, value in enumerate(values_list):
        joint_array[index] = value
    return joint_array


class CartesianKinematics:
    def __init__(
        self,
        *,
        chain: kdl.Chain,
        joint_names: list[str],
        lower_limits: list[float],
        upper_limits: list[float],
        base_link: str,
        tool_link: str,
        orientation_weight: float = 0.15,
        damping: float = 0.05,
        max_iterations: int = 200,
        epsilon: float = 1e-5,
        max_step_norm: float = 0.2,
    ) -> None:
        if len(joint_names) == 0:
            raise ValueError('Expected at least one movable joint in the chain.')
        if len(joint_names) != len(lower_limits) or len(joint_names) != len(upper_limits):
            raise ValueError('Joint limits must match the movable joint count.')
        if orientation_weight <= 0.0:
            raise ValueError('orientation_weight must be positive.')
        if damping <= 0.0:
            raise ValueError('damping must be positive.')
        if max_iterations <= 0:
            raise ValueError('max_iterations must be positive.')
        if epsilon <= 0.0:
            raise ValueError('epsilon must be positive.')
        if max_step_norm <= 0.0:
            raise ValueError('max_step_norm must be positive.')

        self.chain = chain
        self.joint_names = joint_names
        self.lower_limits = np.array(lower_limits, dtype=float)
        self.upper_limits = np.array(upper_limits, dtype=float)
        self.base_link = base_link
        self.tool_link = tool_link
        self.orientation_weight = float(orientation_weight)
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.epsilon = float(epsilon)
        self.max_step_norm = float(max_step_norm)
        self._task_weights = np.diag(
            [1.0, 1.0, 1.0, self.orientation_weight, self.orientation_weight, self.orientation_weight]
        )
        self._fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self._jacobian_solver = kdl.ChainJntToJacSolver(self.chain)

    @classmethod
    def from_urdf_string(
        cls,
        robot_description: str | bytes,
        base_link: str,
        tool_link: str,
        **kwargs,
    ) -> 'CartesianKinematics':
        xml = robot_description.encode() if isinstance(robot_description, str) else robot_description
        return cls.from_urdf_model(URDF.from_xml_string(xml), base_link, tool_link, **kwargs)

    @classmethod
    def from_urdf_model(
        cls,
        robot: URDF,
        base_link: str,
        tool_link: str,
        **kwargs,
    ) -> 'CartesianKinematics':
        child_to_joint = {joint.child: joint for joint in robot.joints}
        if base_link not in robot.link_map:
            raise ValueError(f'Base link {base_link!r} is missing from the URDF.')
        if tool_link not in robot.link_map:
            raise ValueError(f'Tool link {tool_link!r} is missing from the URDF.')

        joint_path: list[Joint] = []
        current_link = tool_link
        while current_link != base_link:
            joint = child_to_joint.get(current_link)
            if joint is None:
                raise ValueError(f'No serial joint path found from {base_link!r} to {tool_link!r}.')
            joint_path.append(joint)
            current_link = joint.parent
        joint_path.reverse()

        chain = kdl.Chain()
        joint_names: list[str] = []
        lower_limits: list[float] = []
        upper_limits: list[float] = []

        for joint in joint_path:
            origin_frame = _urdf_pose_to_kdl_frame(joint.origin)
            kdl_joint = _joint_to_kdl_joint(joint, origin_frame)
            chain.addSegment(kdl.Segment(joint.child, kdl_joint, origin_frame))
            if joint.type in MOVABLE_JOINT_TYPES:
                joint_names.append(joint.name)
                lower_limit, upper_limit = _joint_limits(joint)
                lower_limits.append(lower_limit)
                upper_limits.append(upper_limit)

        return cls(
            chain=chain,
            joint_names=joint_names,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            base_link=base_link,
            tool_link=tool_link,
            **kwargs,
        )

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    def forward_kinematics(self, positions: Iterable[float]) -> kdl.Frame:
        joint_array = _jnt_array(positions)
        if joint_array.rows() != self.joint_count:
            raise ValueError(
                f'Expected {self.joint_count} joint positions, got {joint_array.rows()}.'
            )

        frame = kdl.Frame()
        status = self._fk_solver.JntToCart(joint_array, frame)
        if status < 0:
            raise RuntimeError(f'Forward kinematics failed with error code {status}.')
        return frame

    def solve_pose(self, target_frame: kdl.Frame, seed_positions: Iterable[float]) -> CartesianIkResult | None:
        seed = np.array([float(position) for position in seed_positions], dtype=float)
        if seed.size != self.joint_count:
            raise ValueError(f'Expected {self.joint_count} seed positions, got {seed.size}.')
        if not np.all(np.isfinite(seed)):
            raise ValueError('Seed positions must be finite.')

        positions = np.clip(seed, self.lower_limits, self.upper_limits)
        best_result: CartesianIkResult | None = None

        for iteration in range(1, self.max_iterations + 1):
            current_frame = self.forward_kinematics(positions.tolist())
            error_vector, position_error, orientation_error = self._frame_error(current_frame, target_frame)
            weighted_error = float(np.linalg.norm(self._task_weights @ error_vector))
            current_result = CartesianIkResult(
                positions=positions.tolist(),
                position_error_m=position_error,
                orientation_error_rad=orientation_error,
                weighted_error=weighted_error,
                iterations=iteration,
                converged=weighted_error <= self.epsilon,
            )
            if best_result is None or current_result.weighted_error < best_result.weighted_error:
                best_result = current_result
            if current_result.converged:
                return current_result

            jacobian = self._jacobian_matrix(positions.tolist())
            delta = self._damped_least_squares_step(jacobian, error_vector)
            if not np.all(np.isfinite(delta)):
                return best_result

            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_step_norm:
                delta *= self.max_step_norm / delta_norm
            if delta_norm <= self.epsilon:
                break

            next_positions = np.clip(positions + delta, self.lower_limits, self.upper_limits)
            if np.linalg.norm(next_positions - positions) <= self.epsilon:
                break
            positions = next_positions

        return best_result

    def _jacobian_matrix(self, positions: list[float]) -> np.ndarray:
        joint_array = _jnt_array(positions)
        jacobian = kdl.Jacobian(self.joint_count)
        status = self._jacobian_solver.JntToJac(joint_array, jacobian)
        if status < 0:
            raise RuntimeError(f'Jacobian solver failed with error code {status}.')

        matrix = np.zeros((6, self.joint_count), dtype=float)
        for row in range(jacobian.rows()):
            for column in range(jacobian.columns()):
                matrix[row, column] = jacobian[row, column]
        return matrix

    def _damped_least_squares_step(self, jacobian: np.ndarray, error_vector: np.ndarray) -> np.ndarray:
        weighted_jacobian = self._task_weights @ jacobian
        weighted_error = self._task_weights @ error_vector
        lhs = weighted_jacobian.T @ weighted_jacobian
        lhs += (self.damping * self.damping) * np.eye(self.joint_count)
        rhs = weighted_jacobian.T @ weighted_error
        try:
            return np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            return np.linalg.lstsq(lhs, rhs, rcond=None)[0]

    def _frame_error(self, current_frame: kdl.Frame, target_frame: kdl.Frame) -> tuple[np.ndarray, float, float]:
        twist = kdl.diff(current_frame, target_frame)
        error_vector = np.array(
            [
                twist.vel[0],
                twist.vel[1],
                twist.vel[2],
                twist.rot[0],
                twist.rot[1],
                twist.rot[2],
            ],
            dtype=float,
        )
        return error_vector, float(twist.vel.Norm()), float(twist.rot.Norm())


def _joint_to_kdl_joint(joint: Joint, origin_frame: kdl.Frame) -> kdl.Joint:
    if joint.type == 'fixed':
        return kdl.Joint(joint.name, kdl.Joint.Fixed)

    axis = joint.axis if joint.axis is not None else [1.0, 0.0, 0.0]
    axis_parent = origin_frame.M * kdl.Vector(*xyz_to_float_triplet(axis))
    if joint.type in {'continuous', 'revolute'}:
        return kdl.Joint(joint.name, origin_frame.p, axis_parent, kdl.Joint.RotAxis)
    if joint.type == 'prismatic':
        return kdl.Joint(joint.name, origin_frame.p, axis_parent, kdl.Joint.TransAxis)

    raise ValueError(f'Unsupported joint type {joint.type!r} in the serial chain.')


def _joint_limits(joint: Joint) -> tuple[float, float]:
    if joint.type == 'continuous':
        return -2.0 * math.pi, 2.0 * math.pi
    if joint.limit is None:
        raise ValueError(f'Joint {joint.name!r} is missing limits.')
    return float(joint.limit.lower), float(joint.limit.upper)
