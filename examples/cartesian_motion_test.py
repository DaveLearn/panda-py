
import numpy as np
import panda_py.controllers
import panda_py.generators
import trio

# set all loggers to debug
# logging.basicConfig(level=logging.DEBUG)



async def main():
  r = panda_py.Panda("10.103.1.111")
  up_pose = np.array(
    [
    [ 0.79075446,  0.15035292,  0.5933813 ,  0.35142134],
    [ 0.06341833, -0.98427178,  0.16488524,  0.20362951],
    [ 0.60883945, -0.09275248, -0.78785246,  0.81773236],
    [ 0.        ,  0.        ,  0.        ,  1.        ]]
  )
  pose = np.array(
  [[ 0.99810542, -0.01901075,  0.05851597,  0.5100107 ],
   [-0.01631634, -0.99879966, -0.04618413,  0.12997817],
   [ 0.05932373,  0.04514186, -0.99721758,  0.22003586],
   [ 0.        ,  0.        ,  0.        ,  1.        ]])


  await r.movex([pose, up_pose, pose], speed=0.2)
  await r.movep([0.51, 0.13, 0.22], speed=1.0)
  await r.movepr([-0.2, 0.0, 0.0], speed=1.0)
  await r.movezr(0.1, speed=1.0)


  p1 = [0.51, 0.13, 0.22]
  p2 = [0.51, 0.33, 0.22]
  ps = [p1, p2, p1]

  ctrl = panda_py.generators.CartesianMotionGenerator(keep_running=True)
  r.start_generator(ctrl)
  i = 0
  orientation = r.get_orientation()
  with r.create_context(frequency=2) as ctx:
    while ctx.ok():
      print("Changing waypoint")
      print(ctx.num_ticks)
      if ctx.num_ticks % 1 == 0:
        i = (i + 1) % 2
        waypoint = panda_py.CartesianMotion(ps[i], orientation=orientation)
        ctrl.clear_waypoints()
        ctrl.add_waypoint(waypoint)

  r.stop_generator()

  



if __name__ == "__main__":
  trio.run(main)
