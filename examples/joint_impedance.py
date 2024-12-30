"""
Uses the cartesian impedance controller to create a sinusoidal
end-effector movement along the robot's y-axis.
"""
import sys

import numpy as np

import panda_py
import panda_py.controllers
import panda_py.generators
import gello


joint_signs = [1, -1, 1, 1, 1, 1, 1]
joint_offsets = [0*np.pi/2, 2*np.pi/2, 2*np.pi/2, 2*np.pi/2, 1*np.pi/2, 2*np.pi/2, 2*np.pi/2 -np.pi/4 ]

def create_gello():
    return gello.DynamixelRobot(
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets=joint_offsets,
        joint_signs=joint_signs,
    )


if __name__ == '__main__':
  if len(sys.argv) < 2:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  panda = panda_py.Panda(sys.argv[1])
  panda.move_to_start()
  g = create_gello()
  # if not gello.check_joint_discrepency(panda.q, g.get_joint_state()):
  #   exit()

  panda.move_to_joint_position(g.get_joint_state())
  stiffness = np.array([500, 500, 500, 300, 70, 70, 50])
  ctrl = panda_py.controllers.JointPosition(stiffness=stiffness, filter_coeff=0.1)
  # ctrl.set_jerk_scaling(1.3)
  # ctrl.set_acceleration_scaling(1.6)
  # ctrl.set_max_jerks(np.array([30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]))
  # ctrl.set_max_accelerations(np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]))
  # ctrl.set_max_velocities(np.array([2.17, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610]))

  x0 = panda.get_position()
  q0 = panda.get_orientation()
  runtime = np.pi * 4.0
  runtime = 0
  panda.start_controller(ctrl)

  joint = 0
  with panda.create_context(frequency=10, max_runtime=runtime) as ctx:
    while ctx.ok():
      x_d = x0.copy()
      q = panda.q
      g_q = g.get_joint_state().round(4)
      print(g_q)
      # q[joint] = g_q[0]
      ctrl.set_control(g_q)
  
  panda.stop_controller()
