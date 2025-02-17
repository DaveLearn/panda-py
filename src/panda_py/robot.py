import numpy as np
import trio

from ._core import Panda as _Panda
from ._core import JointMotion, CartesianMotion, ReferenceFrame
from .generators import JointMotionGenerator, Generator, CartesianMotionGenerator


class Panda(_Panda):
    async def move_to_start(self):
        q = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
        await self.movej(q)

    async def rotate_to_start(self):
        r = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float32)
        await self.mover(r)

    async def movezr(self, distance: float, speed: float = 1.0):
        await self.movep(
            [0.0, 0.0, distance], speed=speed, reference_frame=ReferenceFrame.RELATIVE
        )

    async def movepr(self, positions: np.ndarray, speed: float | list[float] = 1.0):
        await self.movep(
            positions, speed=speed, reference_frame=ReferenceFrame.RELATIVE
        )

    async def mover(
        self,
        rotations: np.ndarray,
        speed: float | list[float] = 1.0,
        reference_frame: ReferenceFrame = ReferenceFrame.GLOBAL,
    ):
        rotations = np.asarray(rotations)
        assert (rotations.ndim == 3 and rotations.shape[1:] == (3, 3)) or (
            rotations.ndim == 2 and rotations.shape == (3, 3)
        ), "rotations must be a 3x3 matrix or a 3D array with 3x3 dimensions"

        if reference_frame == ReferenceFrame.RELATIVE:
            current_position = [0.0, 0.0, 0.0]
        else:
            current_position = self.get_position()

        if rotations.ndim == 2:
            rotations = rotations.reshape(1, 3, 3)

        poses = []
        for r in rotations:
            pose = np.eye(4)
            pose[:3, 3] = current_position
            pose[:3, :3] = r
            poses.append(pose)

        await self.movex(poses, speed=speed, reference_frame=reference_frame)

    async def movep(
        self,
        positions: np.ndarray,
        speed: float | list[float] = 1.0,
        reference_frame: ReferenceFrame = ReferenceFrame.GLOBAL,
    ):
        positions = np.asarray(positions)
        assert (positions.ndim == 2 and positions.shape[1] == 3) or (
            positions.ndim == 1 and positions.shape[0] == 3
        ), "positions must be a 3D vector or a 2D array with 3 columns"
        if reference_frame == ReferenceFrame.RELATIVE:
            current_rotation = np.eye(3)
        else:
            current_pose = self.get_pose()
            current_rotation = current_pose[:3, :3]

        if positions.ndim == 1:
            positions = positions.reshape(1, 3)

        poses = []
        for p in positions:
            pose = np.eye(4)
            pose[:3, 3] = p
            pose[:3, :3] = current_rotation
            poses.append(pose)

        await self.movex(poses, speed=speed, reference_frame=reference_frame)

    async def movex(
        self,
        pose: np.ndarray,
        speed: float | list[float] = 1.0,
        reference_frame: ReferenceFrame = ReferenceFrame.GLOBAL,
    ):
        pose = np.asarray(pose)
        assert pose.shape == (4, 4) or (pose.ndim == 3 and pose.shape[1:] == (4, 4)), (
            "pose must be a 4x4 matrix or a 3D array with 4x4 dimensions"
        )

        if pose.ndim == 2:
            pose = pose.reshape(1, 4, 4)

        num_waypoints = pose.shape[0]

        if isinstance(speed, float):
            speed = [speed] * num_waypoints

        cartesian_motions = []
        for i, p in enumerate(pose):
            cartesian_motion = CartesianMotion(p, reference_frame=reference_frame)
            cartesian_motion.velocity_rel = speed[i]
            cartesian_motion.jerk_rel = 0.1
            cartesian_motions.append(cartesian_motion)

        ctrl = CartesianMotionGenerator(keep_running=False)
        ctrl.add_waypoints(cartesian_motions)
        await self._run_generator(ctrl)
        self.raise_error()

    async def movej(self, joints: np.ndarray, speed: float | list[float] = 1.0):
        joints = np.asarray(joints)
        assert joints.shape == (7,) or (joints.ndim == 2 and joints.shape[1] == 7), (
            "joints must be a 7D vector or a 2D array with 7 columns"
        )

        if joints.ndim == 1:
            joints = joints.reshape(1, 7)

        num_waypoints = joints.shape[0]

        if isinstance(speed, float):
            speed = [speed] * num_waypoints

        joint_motions = []
        for i, j in enumerate(joints):
            joint_motion = JointMotion(j)
            joint_motion.velocity_rel = speed[i]
            joint_motions.append(joint_motion)

        ctrl = JointMotionGenerator(keep_running=False)
        ctrl.add_waypoints(joint_motions)
        await self._run_generator(ctrl)
        self.raise_error()

    async def _run_generator(self, ctrl: Generator):
        current_token = trio.lowlevel.current_trio_token()
        current_task = trio.lowlevel.current_task()

        def abort_func(raise_cancel):
            self.stop_generator()
            return trio.lowlevel.Abort.FAILED

        def done_callback():
            current_token.run_sync_soon(trio.lowlevel.reschedule, current_task)

        ctrl.set_done_callback(done_callback)

        self.start_generator(ctrl)
        await trio.lowlevel.wait_task_rescheduled(abort_func=abort_func)
