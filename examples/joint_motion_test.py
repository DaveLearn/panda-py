
import numpy as np
import panda_py.controllers
import panda_py.generators
import trio


async def main():
  r = panda_py.Panda("10.103.1.111")
  q1 = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4]) # home
  q2 = [-1.02657695, -1.42662793,  1.47967159, -2.37939987,  1.4366989,   1.63783955,
    0.54400868] 
  
  async def check_moving():
    while True:
      print(r.is_moving())
      trio.sleep(0.1)

  async with trio.open_nursery() as nursery:
    # nursery.start_soon(check_moving)
    await r.movej([q2, q1], speed=[0.1, 1.0])
    nursery.cancel_scope.cancel()
  
  



if __name__ == "__main__":
  trio.run(main)
