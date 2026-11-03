"""四旋翼 3D 仿真：MuJoCo 刚体物理 + 自写旋翼气动与串级控制。

与 `skyfleet/`（通信事件仿真）刻意解耦：那边研究的是协议与网络，这边研究
的是飞行器本身的动力学与控制，两边不互相 import。

    from flightlab.sim import QuadrotorSim
    sim = QuadrotorSim()
    sim.reset(pos=(0, 0, 0.05))
    log = sim.run(10.0, (0, 0, 1.5))
"""
__all__ = ["sim", "control", "params", "tuning", "render", "cli"]
