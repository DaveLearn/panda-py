"""

Control library for the Panda robot. The general workflow
is to instantiate a controller and hand it over to the
:py:class:`panda_py.Panda` class for execution using the
function :py:func:`panda_py.Panda.start_controller`.
"""
from __future__ import annotations
from panda_py._core import CartesianMotionGenerator
from panda_py._core import Generator
from panda_py._core import JointMotionGenerator
__all__ = ['CartesianMotionGenerator', 'Generator', 'JointMotionGenerator']
