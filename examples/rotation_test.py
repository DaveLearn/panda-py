
import numpy as np
import panda_py.controllers
import panda_py.generators
import trio

async def main():
  r = panda_py.Panda("10.103.1.111")
  rotation = np.array(
    [[1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]], dtype=np.float32)
  
  await r.mover(rotation, speed=0.1)



if __name__ == "__main__":
  trio.run(main)
