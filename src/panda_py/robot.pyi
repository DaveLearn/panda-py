from __future__ import annotations
import numpy
import numpy as np
import panda_py._core
from panda_py._core import CartesianMotion
from panda_py._core import CartesianMotionGenerator
from panda_py._core import Generator
from panda_py._core import JointMotion
from panda_py._core import JointMotionGenerator
from panda_py._core import Panda as _Panda
from panda_py._core import ReferenceFrame
import trio as trio
__all__ = ['CartesianMotion', 'CartesianMotionGenerator', 'Generator', 'JointMotion', 'JointMotionGenerator', 'Panda', 'ReferenceFrame', 'np', 'trio']
class Panda(panda_py._core.Panda):
    def _run_generator(self, ctrl: panda_py._core.Generator):
        ...
    def movej(self, joints: numpy.ndarray, speed: float | list[float] = 1.0):
        ...
    def movep(self, positions: numpy.ndarray, speed: float | list[float] = 1.0, reference_frame: panda_py._core.ReferenceFrame = ...):
        ...
    def movepr(self, positions: numpy.ndarray, speed: float | list[float] = 1.0):
        ...
    def mover(self, rotations: numpy.ndarray, speed: float | list[float] = 1.0, reference_frame: panda_py._core.ReferenceFrame = ...):
        ...
    def movex(self, pose: numpy.ndarray, speed: float | list[float] = 1.0, reference_frame: panda_py._core.ReferenceFrame = ...):
        ...
    def movezr(self, distance: float, speed: float = 1.0):
        ...
