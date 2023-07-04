#!/usr/bin/python

import sys
import time
import math

# import numpy as np

sys.path.append("../lib/python/amd64")
import robot_interface as sdk
from pdb import set_trace


def jointLinearInterpolation(initPos, targetPos, rate):
    # rate = np.fmin(np.fmax(rate, 0.0), 1.0)

    if rate > 1.0:
        rate = 1.0
    elif rate < 0.0:
        rate = 0.0

    p = initPos * (1 - rate) + targetPos * rate
    return p


if __name__ == "__main__":
    d = {
        "FR_0": 0,
        "FR_1": 1,
        "FR_2": 2,
        "FL_0": 3,
        "FL_1": 4,
        "FL_2": 5,
        "RR_0": 6,
        "RR_1": 7,
        "RR_2": 8,
        "RL_0": 9,
        "RL_1": 10,
        "RL_2": 11,
    }
    PosStopF = math.pow(10, 9)
    VelStopF = 16000.0
    HIGHLEVEL = 0xEE
    LOWLEVEL = 0xFF
    sin_mid_q = [0.0, 1.2, -2.0]
    dt = 0.002
    qInit = [0, 0, 0]
    qDes = [0, 0, 0]
    sin_count = 0
    rate_count = 0
    Kp = [0, 0, 0]
    Kd = [0, 0, 0]

    # udp = sdk.UDP(8080, "192.168.123.10", 8007, 614, 807, False, sdk.RecvEnum.nonBlock)
    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    safe = sdk.Safety(sdk.LeggedType.B1)

    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)

    Tpi = 0
    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime += 1

        udp.Recv()
        udp.GetRecv(state)

        print(
            "FR0: {:.4f} \t FL0: {:.4f} \t RR0: {:.4f} \t RL0: {:.4f}".format(
                state.motorState[d["FR_0"]].q,
                state.motorState[d["FL_0"]].q,
                state.motorState[d["RR_0"]].q,
                state.motorState[d["RL_0"]].q,
            )
        )
        # print(
        #     "FR1: {:.4f} \t FL1: {:.4f} \t RR1: {:.4f} \t RL1: {:.4f}".format(
        #         state.motorState[d["FR_1"]].q,
        #         state.motorState[d["FL_1"]].q,
        #         state.motorState[d["RR_1"]].q,
        #         state.motorState[d["RL_1"]].q,
        #     )
        # )
        # print(
        #     "FR2: {:.4f} \t FL2: {:.4f} \t RR2: {:.4f} \t RL2: {:.4f}".format(
        #         state.motorState[d["FR_2"]].q,
        #         state.motorState[d["FL_2"]].q,
        #         state.motorState[d["RR_2"]].q,
        #         state.motorState[d["RL_2"]].q,
        #     )
        # )
