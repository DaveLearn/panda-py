import numpy as np
import panda_py.controllers
import panda_py.generators
import trio


async def main():
  r = panda_py.Panda("10.103.1.111")

  async def read_q():
    while True:
      # q = r.get_robot().read_once().q
      print(r.q)
      # print(q)
      await trio.sleep(0.1)

  print("q:", r.q)
  async with trio.open_nursery() as nursery:
    await read_q()


if __name__ == "__main__":
  trio.run(main)
